"""
Automatic data labelling pipeline with SAM 3.

Iterates through all videos in videos/ (ignores CSVs).
For each frame:
  1. CLIP (check_road.has_road) decides if a road is visible ahead.
  2. If road: run old segmentation model (model_v2.Onnx) to get a prior mask.
  3. Binary-search the largest square fully inside the prior mask (integral image),
     then derive 4 points at the corners of a smaller concentric square.
  4. Invoke SAM 3 with BOTH text ("road ahead including checkered start
     finish line", so a checkered start/finish line is segmented as part
     of the road) and those 4 point prompts.
  5. Save label mask to labels/, the raw frame to labeled/ (training input,
     paired by filename), and the mask overlay to overlay/ (visualisation).

The whole video is processed so road appear/disappear cycles are all captured.

Autoresume: progress is tracked in .finished.txt (finished videos, plus the
current video and frame offset). Resubmit the same command after an
interruption and it continues where it left off. Video order is sorted so
it is deterministic across runs. Do not share one state file/output dirs
between concurrent jobs (use --state / separate folders per job).

Usage (conda env Distill_v2):
    conda activate Distill_v2
    python main_sam3.py [--videos videos] [--labels labels] [--labeled labeled]
                        [--overlay overlay] [--stride 5] [--max-frames 20]
                        [--video NAME.mp4] [--clip-threshold 0.15] [--sam-conf 0.5]

SAM 3 checkpoints are gated. Before first use:
    hf auth login   # with access granted to facebook/sam3
"""
import argparse
import os
import glob
import cv2
import numpy as np
import torch
from PIL import Image

from check_road import has_road
from model_v2 import Onnx

TEXT_PROMPT = "road ahead including checkered start finish line"


# ---------------------------------------------------------------------------
# Largest inscribed square via binary search (integral image, O(1) rect query)
# ---------------------------------------------------------------------------
def four_points_in_mask(mask):
    """Find largest square inside binary mask, return 4 interior points.

    mask: uint8 HxW, values 0 or 255.
    returns: list of 4 (x, y) pixel coords, or None if mask is empty.
    """
    binary = (mask > 0).astype(np.uint8)
    h, w = binary.shape
    if np.count_nonzero(binary) == 0:
        return None

    integral = cv2.integral(binary)

    def rect_sum(x0, y0, x1, y1):
        return (integral[y1, x1] - integral[y0, x1]
                - integral[y1, x0] + integral[y0, x0])

    def square_exists(side):
        area = side * side
        # stride the scan for speed on 720p (step ~ side/8, then refine centre)
        # full exact scan is O(H*W*log) which is fine at 720p; keep exact.
        for y in range(0, h - side + 1):
            for x in range(0, w - side + 1):
                if rect_sum(x, y, x + side, y + side) == area:
                    return True, x + side // 2, y + side // 2
        return False, None, None

    max_side = min(h, w)
    if max_side % 2 == 0:
        max_side -= 1

    low, high = 1, max_side
    best_center, best_side = None, 1
    while low <= high:
        mid = (low + high) // 2
        if mid % 2 == 0:
            mid -= 1
        if mid < 1:
            break
        found, cx, cy = square_exists(mid)
        if found:
            best_side = mid
            best_center = (cx, cy)
            low = mid + 2
        else:
            high = mid - 2

    if best_center is None:
        ys, xs = np.where(binary > 0)
        if len(xs) == 0:
            return None
        cx, cy = int(xs.mean()), int(ys.mean())
        return [(cx, cy)] * 4

    cx, cy = best_center
    sixth = max(1, round(best_side * 1 / 6))
    pts = [(cx + sixth, cy + sixth), (cx + sixth, cy - sixth),
           (cx - sixth, cy + sixth), (cx - sixth, cy - sixth)]
    # clamp + verify inside mask; fall back to centre if a corner leaks out
    out = []
    for (x, y) in pts:
        x = min(max(x, 0), w - 1)
        y = min(max(y, 0), h - 1)
        if binary[y, x] == 0:
            x, y = cx, cy
        out.append((x, y))
    print(f"Largest square side: {best_side}, center: {(cx, cy)}, points: {out}",
          flush=True)
    return out


# ---------------------------------------------------------------------------
# Autoresume state (.finished.txt)
#
# Tracks finished videos plus the in-progress video and resume offset, so
# interrupted cluster jobs can continue where they left off by resubmitting
# the same command. File format (one entry per line):
#   done <video_basename>        # fully processed video
#   current <video_basename> <next_frame_idx>   # resume reading at this frame
# Writes are atomic (temp file + rename) to survive kills mid-write.
# NOTE: do not run concurrent jobs sharing one state file / output dirs.
# ---------------------------------------------------------------------------
def load_state(path):
    """returns: (done_list_sorted, current_video_or_None, current_next_idx)."""
    done, cur_video, cur_next = [], None, 0
    if not os.path.isfile(path):
        return done, cur_video, cur_next
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if parts[0] == "done" and len(parts) == 2:
                done.append(parts[1])
            elif parts[0] == "current" and len(parts) == 3:
                try:
                    cur_video, cur_next = parts[1], int(parts[2])
                except ValueError:
                    continue
    return sorted(set(done)), cur_video, max(cur_next, 0)


def save_state(path, done_list, current_video, next_idx):
    """Atomically rewrite the state file. done_list kept sorted/deduped."""
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        f.write("# autoresume state for main_sam3.py — "
                "do not edit while a job is running\n")
        for v in sorted(set(done_list)):
            f.write(f"done {v}\n")
        if current_video is not None:
            f.write(f"current {current_video} {int(next_idx)}\n")
    os.replace(tmp, path)


def create_label_png(mask):
    """Background black, road gray (127,127,127). mask: HxW 0/255."""
    h, w = mask.shape
    out = np.zeros((h, w, 3), dtype=np.uint8)
    out[mask > 0] = (127, 127, 127)
    return out


def overlay_mask(image_bgr, mask, alpha=0.4):
    result = image_bgr.copy()
    mb = mask > 0
    blue = np.zeros_like(image_bgr)
    blue[:, :] = (255, 0, 0)
    result[mb] = cv2.addWeighted(image_bgr[mb], 1 - alpha, blue[mb], alpha, 0)
    return result


# ---------------------------------------------------------------------------
# SAM 3 wrapper: text + point prompts (native facebookresearch backend)
# ---------------------------------------------------------------------------
class Sam3Labeler:
    def __init__(self, device="cuda", conf=0.5):
        from sam3.model_builder import build_sam3_image_model
        from sam3.model.sam3_image_processor import Sam3Processor
        torch.backends.cuda.matmul.allow_tf32 = True
        torch.backends.cudnn.allow_tf32 = True
        self.device = device if torch.cuda.is_available() else "cpu"
        print(f"Loading SAM 3 on {self.device} ...", flush=True)
        print("NOTE: checkpoints are gated — run `hf auth login` with "
              "access to facebook/sam3 first.", flush=True)
        self.model = build_sam3_image_model(device=self.device)
        self.processor = Sam3Processor(self.model, device=self.device,
                                       confidence_threshold=conf)
        print("SAM 3 loaded.", flush=True)

    def predict(self, image_bgr, points_xy, text=TEXT_PROMPT):
        """Run SAM 3 with text + 4 point prompts.

        image_bgr: HxWx3 BGR uint8 (full res, coords of points refer to this).
        points_xy: list of 4 (x, y).
        returns: HxW uint8 mask (0/255) at input resolution.
        """
        image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
        pil = Image.fromarray(image_rgb)
        W, H = pil.size
        use_bf16 = self.device == "cuda"
        ctx = (torch.autocast("cuda", dtype=torch.bfloat16)
               if use_bf16 else torch.cpu.amp.autocast(enabled=False))
        with ctx:
            state = self.processor.set_image(pil)
            state = self.processor.set_text_prompt(prompt=text, state=state)
            # Inject the 4 point prompts (normalized xy in [0,1]).
            pts_norm = [[min(max(x / W, 0.0), 1.0),
                         min(max(y / H, 0.0), 1.0)] for (x, y) in points_xy]
            n = len(pts_norm)
            pts_t = torch.tensor(pts_norm, dtype=torch.float32,
                                 device=self.device).view(n, 1, 2)
            lbl_t = torch.ones(n, 1, dtype=torch.long, device=self.device)
            try:
                state["geometric_prompt"].append_points(pts_t, lbl_t)
                state = self.processor._forward_grounding(state)
            except Exception as e:
                print(f"append_points failed ({e}); falling back to box prompts",
                      flush=True)
                xs = [p[0] for p in points_xy]
                ys = [p[1] for p in points_xy]
                cx = (min(xs) + max(xs)) / 2 / W
                cy = (min(ys) + max(ys)) / 2 / H
                bw = max(max(xs) - min(xs), 8) / W
                bh = max(max(ys) - min(ys), 8) / H
                state = self.processor.add_geometric_prompt(
                    [cx, cy, bw, bh], True, state)
            masks = state["masks"]    # bool tensor
            scores = state["scores"]
        if masks is None or masks.numel() == 0:
            return np.zeros((H, W), dtype=np.uint8)
        if scores is not None and scores.numel() > 1:
            best = int(torch.argmax(scores).item())
        else:
            best = 0
        m = masks[best]
        if torch.is_tensor(m):
            m = m.detach().cpu().numpy()
        # masks come back as (N, 1, H, W); drop singleton dims -> (H, W)
        m = np.squeeze(m)
        m = (m > 0.5).astype(np.uint8) * 255
        if m.shape != (H, W):
            m = cv2.resize(m, (W, H), interpolation=cv2.INTER_NEAREST)
        kernel = np.ones((5, 5), np.uint8)
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, kernel)
        return m


def process_video(video_path, old_model, sam3, args, stats,
                  start_idx=0, progress_cb=None):
    video_name = os.path.splitext(os.path.basename(video_path))[0]
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"Cannot open {video_path}, skipping")
        return False
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    frame_idx = 0
    if start_idx > 0:
        cap.set(cv2.CAP_PROP_POS_FRAMES, start_idx)
        frame_idx = start_idx
        print(f"\n=== {video_name}: {total} frames (resuming at {start_idx}) ===",
              flush=True)
    else:
        print(f"\n=== {video_name}: {total} frames ===", flush=True)
    in_road = False       # for appearance/disappearance logging
    appearances = 0
    saved = 0
    completed = True
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        idx = frame_idx
        frame_idx += 1
        if idx % args.stride != 0:
            continue
        if args.max_frames is not None and saved >= args.max_frames:
            print(f"  max-frames {args.max_frames} reached for this video")
            completed = False
            break
        if progress_cb is not None:
            # frames [0, frame_idx) consumed; resume seeks here on restart
            progress_cb(frame_idx)
        # 1. CLIP road check on full-res frame
        try:
            road = has_road(frame, args.clip_threshold)
        except Exception as e:
            print(f"  frame {idx}: CLIP error {e}, skipping")
            continue
        if not road:
            if in_road:
                print(f"  frame {idx}: road disappeared "
                      f"(appearance #{appearances} ended)", flush=True)
            in_road = False
            continue
        if not in_road:
            appearances += 1
            print(f"  frame {idx}: road appearance #{appearances} begins",
                  flush=True)
        in_road = True
        # 2. Old model prior mask (model_v2 expects 640x352 input)
        try:
            H0, W0 = frame.shape[:2]
            small = cv2.resize(frame, (640, 352), interpolation=cv2.INTER_LINEAR)
            prior_small = old_model.run(small)
            prior_small = (prior_small > 100).astype(np.uint8) * 255
            prior = cv2.resize(prior_small, (W0, H0),
                               interpolation=cv2.INTER_NEAREST)
        except Exception as e:
            print(f"  frame {idx}: old-model error {e}, skipping")
            continue
        if np.count_nonzero(prior) == 0:
            print(f"  frame {idx}: road present (CLIP) but prior mask empty, "
                  f"skipping SAM 3")
            continue
        points = four_points_in_mask(prior)
        if points is None:
            print(f"  frame {idx}: no points from prior mask, skipping")
            continue
        # 3. SAM 3 with text + points (or prior-mask fallback for testing).
        # The text prompt names the checkered start/finish line so SAM
        # segments it as part of the road; the 4 point prompts from the
        # teacher mask ground the "road ahead" part of the phrase.
        if args.no_sam3:
            sam_mask = prior
        else:
            try:
                sam_mask = sam3.predict(frame, points, text=args.text)
            except Exception as e:
                print(f"  frame {idx}: SAM 3 error {e}, skipping")
                continue
        if np.count_nonzero(sam_mask) == 0:
            print(f"  frame {idx}: SAM 3 returned empty mask, skipping save")
            continue
        # 4. Save label + raw input frame + overlay.
        # labels/: gray road mask (training target).
        # labeled/: raw frame (training input, paired by filename with labels/).
        # overlay/: mask overlay for visual inspection.
        base = f"{video_name}_{idx:06d}"
        cv2.imwrite(os.path.join(args.labels, base + ".png"),
                    create_label_png(sam_mask))
        cv2.imwrite(os.path.join(args.labeled, base + ".jpg"), frame)
        if args.save_overlay:
            vis = overlay_mask(frame, sam_mask)
            cv2.imwrite(os.path.join(args.overlay, base + ".jpg"), vis)
        saved += 1
        print(f"  frame {idx}: saved {base} "
              f"(road px: {np.count_nonzero(sam_mask)})", flush=True)
    cap.release()
    print(f"=== {video_name} done: {saved} labelled frames, "
          f"{appearances} road appearance(s) ===", flush=True)
    stats["videos"] += 1
    stats["frames_saved"] += saved
    stats["appearances"] += appearances
    return completed


def main():
    ap = argparse.ArgumentParser(description="SAM 3 road distillation pipeline")
    ap.add_argument("--videos", default="videos")
    ap.add_argument("--labels", default="labels")
    ap.add_argument("--labeled", default="labeled",
                    help="folder for raw frames paired with labels "
                         "(training inputs)")
    ap.add_argument("--overlay", default="overlay",
                    help="folder for mask overlays (visualisation)")
    ap.add_argument("--stride", type=int, default=5,
                    help="process every Nth frame (default 5)")
    ap.add_argument("--max-frames", type=int, default=None,
                    help="max labelled frames per video (default: no limit)")
    ap.add_argument("--video", default=None,
                    help="process only this video file (basename or path)")
    ap.add_argument("--clip-threshold", type=float, default=0.0,
                    help="CLIP has_road threshold (0.0 matches Distill_Sam; "
                         "0.15 default in check_road filters almost everything)")
    ap.add_argument("--sam-conf", type=float, default=0.5)
    ap.add_argument("--text", default=TEXT_PROMPT)
    ap.add_argument("--save-overlay", action="store_true", default=True)
    ap.add_argument("--no-overlay", dest="save_overlay", action="store_false",
                    help="skip saving mask overlays to overlay/")
    ap.add_argument("--onnx", default="model_v2.onnx")
    ap.add_argument("--no-sam3", action="store_true",
                    help="testing only: use prior mask as label, skip SAM 3 "
                         "(for I/O checks without gated checkpoint)")
    ap.add_argument("--state", default=".finished.txt",
                    help="autoresume state file: finished videos + current "
                         "video and frame offset (default .finished.txt)")
    ap.add_argument("--fresh", action="store_true",
                    help="ignore existing state file and process from scratch")
    args = ap.parse_args()

    os.makedirs(args.labels, exist_ok=True)
    os.makedirs(args.labeled, exist_ok=True)
    os.makedirs(args.overlay, exist_ok=True)

    # video list: only .mp4, ignore csvs — sorted for deterministic order
    # across runs (required for autoresume to make sense).
    if args.video:
        cand = args.video if os.path.isfile(args.video) else os.path.join(
            args.videos, args.video)
        video_files = sorted([cand]) if os.path.isfile(cand) else sorted(
            glob.glob(os.path.join(args.videos, "*.mp4")))
        if not os.path.isfile(cand):
            print(f"{cand} not found, processing all videos instead")
    else:
        video_files = sorted(glob.glob(os.path.join(args.videos, "*.mp4")))
    print(f"Found {len(video_files)} videos (csvs ignored)")

    # CLIP loads at import of check_road; old model:
    print("Loading old segmentation model ...", flush=True)
    # model_v2.Onnx hardcodes ./model_v2.onnx; patch path if needed
    import model_v2
    if args.onnx != "model_v2.onnx" or not os.path.isfile("./model_v2.onnx"):
        # monkey-patch: temporarily chdir-independent load via absolute path
        import onnxruntime as rt
        orig_init = model_v2.Onnx.__init__
        onnx_path = (args.onnx if os.path.isfile(args.onnx)
                     else os.path.join(os.path.dirname(__file__), "model_v2.onnx"))

        def patched_init(self, p=onnx_path):
            self.sess = rt.InferenceSession(
                p, providers=[("CUDAExecutionProvider",
                               {"cudnn_conv_algo_search": "EXHAUSTIVE"}),
                              "CPUExecutionProvider"])
            print(f"model loaded from {p}")
        model_v2.Onnx.__init__ = patched_init
    try:
        old_model = Onnx()
    except Exception as e:
        print(f"CUDA provider failed ({e}), retrying CPU")
        import onnxruntime as rt
        old_model = Onnx.__new__(Onnx)
        old_model.sess = rt.InferenceSession(args.onnx,
                                             providers=["CPUExecutionProvider"])

    sam3 = None if args.no_sam3 else Sam3Labeler(conf=args.sam_conf)

    # Autoresume: skip finished videos, seek into the in-progress one.
    if args.fresh:
        done, cur_video, cur_next = [], None, 0
        print("Ignoring state file (--fresh), starting from scratch")
    else:
        done, cur_video, cur_next = load_state(args.state)
        if done or cur_video is not None:
            print(f"Resuming from {args.state}: {len(done)} video(s) done"
                  + (f", {cur_video} at frame {cur_next}"
                     if cur_video is not None else ""))
    done_set = set(done)

    stats = {"videos": 0, "frames_saved": 0, "appearances": 0}
    for vp in video_files:
        base = os.path.basename(vp)
        if base in done_set:
            print(f"Skipping finished video: {base}")
            continue
        start_idx = cur_next if base == cur_video else 0
        save_state(args.state, done, base, start_idx)
        completed = process_video(
            vp, old_model, sam3, args, stats, start_idx=start_idx,
            progress_cb=lambda nxt, b=base: save_state(args.state, done,
                                                       b, nxt))
        if completed:
            done.append(base)
            done = sorted(set(done))
            done_set = set(done)
            cur_video, cur_next = None, 0
            save_state(args.state, done, None, 0)
        else:
            # Interrupted (e.g. --max-frames cap): state already holds the
            # resume offset; the next run continues from there.
            pass
    print(f"\nDONE: {stats['videos']} videos, "
          f"{stats['appearances']} road appearances, "
          f"{stats['frames_saved']} labelled frames")


if __name__ == "__main__":
    main()
