import torch
from PIL import Image
import clip


device = "cuda" if torch.cuda.is_available() else "cpu"

model, preprocess = clip.load("ViT-B/32", device=device)

positive_prompts = [
    "a road directly in front of the camera",
    "a road in the foreground",
    "a road extending from below the camera",
    "a close road visible to the side of the viewer",
    "a close road the corner of the camera in a grassy field",
]

negative_prompts = [
    "a grass field with no road",
    "a grass field with some road in the distance",
    "a field with no drivable path in the foreground",
    "a stone structure with no road in the foreground",
    "a metal structure with no road in the foreground",
    "plastic traffic barriers in the foreground",
    "a staircase in a grass field",
    "tables and chairs"
]


text = clip.tokenize(
    positive_prompts + negative_prompts
).to(device)

with torch.no_grad():
    text_features = model.encode_text(text)
    text_features /= text_features.norm(dim=-1, keepdim=True)


def has_road(image, threshold=0.15):
    """
    Returns True if CLIP thinks a road is visible
    in the foreground of the image.

    threshold:
        difference between positive and negative scores
        typical range: 0.05 - 0.3
    """

    image = Image.fromarray(image[:, :, ::-1])
    image = preprocess(image).unsqueeze(0).to(device)

    with torch.no_grad():
        image_features = model.encode_image(image)
        image_features /= image_features.norm(dim=-1, keepdim=True)

        similarities = (
            image_features @ text_features.T
        )[0]

    pos_score = similarities[:len(positive_prompts)].max().item()
    neg_score = similarities[len(positive_prompts):].max().item()
        
    #print(f"pos_score: {pos_score}")
    #print(f"neg_score: {neg_score}")
    return (pos_score - neg_score) > threshold
