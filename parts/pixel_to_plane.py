import numpy as np
import cv2
import pyzed.sl as sl

class PixelToPlane:
    """
    Uses zed.find_plane_at_hit(u,v) to get a plane, then overlays that plane on the left image
    using the point cloud (per-pixel XYZ).

    Inputs:
      - left: np.ndarray (H,W,4 BGRA) or (H,W,3)
      - point_cloud: sl.Mat or np.ndarray(H,W,4) where [:,:,:3] are X,Y,Z
      - u, v: pixel coordinate (ints)

    Outputs:
      - left_overlay: np.ndarray (H,W,3) BGR
      - plane_eq: (a,b,c,d) or None
      - plane_normal: (nx,ny,nz) or None
    """

    def __init__(self, zed: sl.Camera, dist_thresh=1.0, alpha=0.35):
        self.zed = zed
        self.dist_thresh = float(dist_thresh)
        self.alpha = float(alpha)

        self.hit_plane = sl.Plane()
        self.pose = sl.Pose()

    def run(self, left, point_cloud, u, v):
        # ---- image to BGR ----
        if left.shape[2] == 4:
            img = cv2.cvtColor(left, cv2.COLOR_BGRA2BGR)
        else:
            img = left.copy()

        # ---- must have a pixel ----
        if u is None or v is None:
            print("No pixel found")
            return img, None, None

        # ---- tracking must be OK for plane-at-hit ----
        tracking_state = self.zed.get_position(self.pose)
        if tracking_state != sl.POSITIONAL_TRACKING_STATE.OK:
            return img, None, None

        # ---- ask ZED for plane at (u,v) ----
        coord = (u,v)
        status = self.zed.find_plane_at_hit(coord, self.hit_plane)
        if status != sl.ERROR_CODE.SUCCESS:
            return img, None, None

        eq = self.hit_plane.get_plane_equation()
        n = self.hit_plane.get_normal()
        plane_eq = (float(eq[0]), float(eq[1]), float(eq[2]), float(eq[3]))
        plane_normal = (float(n[0]), float(n[1]), float(n[2]))


        print("plane_eq:", plane_eq)
        print("plane_normal:", plane_normal)

        # ---- overlay using point cloud ----
        if isinstance(point_cloud, sl.Mat):
            pc = np.array(point_cloud.get_data())
        else:
            pc = point_cloud

        if pc is None or pc.ndim != 3 or pc.shape[2] < 3:
            return img, plane_eq, plane_normal

        xyz = pc[..., :3].astype(np.float32)
        X, Y, Z = xyz[..., 0], xyz[..., 1], xyz[..., 2]
        valid = np.isfinite(X) & np.isfinite(Y) & np.isfinite(Z)

        a, b, c, d = plane_eq
        denom = (a*a + b*b + c*c) ** 0.5
        if denom < 1e-8:
            return img, plane_eq, plane_normal

        # signed distance, since we normalize
        dist = (a*X + b*Y + c*Z + d) / denom
        mask = valid & (np.abs(dist) < self.dist_thresh)

        overlay = img.copy()
        overlay[mask] = (0, 255, 0)
        out = cv2.addWeighted(img, 1.0 - self.alpha, overlay, self.alpha, 0.0)

        # mark the clicked pixel
        cv2.circle(out, (u, v), 4, (0, 0, 255), -1)

        return out, plane_eq, plane_normal
