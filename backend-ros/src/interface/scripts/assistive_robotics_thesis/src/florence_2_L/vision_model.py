### vision_model.py

from transformers import AutoProcessor, AutoModelForCausalLM
from PIL import Image
import requests
import numpy as np
import cv2
import torch
import time
import matplotlib.pyplot as plt
import matplotlib.patches as patches

def init():
    """Initialize Florence-2-large model and processor."""
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    torch_dtype = torch.float16 if torch.cuda.is_available() else torch.float32

    model = AutoModelForCausalLM.from_pretrained(
        "microsoft/Florence-2-large", 
        torch_dtype=torch_dtype, 
        trust_remote_code=True
    ).to(device).eval()

    processor = AutoProcessor.from_pretrained(
        "microsoft/Florence-2-large", 
        trust_remote_code=True
    )

    return model, processor


def run_example(model, processor, task_prompt=None, text_input=None, image=None):
    """Run Florence-2-large inference and return bounding box results."""
    start_total = time.time()

    # Build prompt
    prompt = task_prompt if text_input is None else task_prompt + text_input
    
    # Preprocessing
    start = time.time()

    inputs = processor(text=prompt, images=image, return_tensors="pt")
    inputs["input_ids"] = inputs["input_ids"].to("cuda")
    inputs["pixel_values"] = inputs["pixel_values"].to("cuda", dtype=torch.float16)
    


    # Inference
    start = time.perf_counter()

    model.eval()
    with torch.inference_mode():
        generated_ids = model.generate(
            input_ids=inputs["input_ids"],
            pixel_values=inputs["pixel_values"],
            max_new_tokens=64,
            early_stopping=False,
            do_sample=False,
            num_beams=1
        )



    # Post-processing (decode + parse)
    start = time.time()
    
    generated_text = processor.batch_decode(generated_ids, skip_special_tokens=False)[0]

    try:
        parsed_answer = processor.post_process_generation(
            generated_text,
            task=task_prompt,
            image_size=(image.shape[1], image.shape[0])
        )
    except Exception as e:
        print("Error during post-processing:", e)
        print("Generated text:", repr(generated_text))
        return {}, generated_text

    return parsed_answer, generated_text


def draw_bounding_boxes_on_rgb(image, bboxes, labels):
    """Draw bounding boxes and labels on RGB image."""
    image_with_bbox = image.copy()
    for bbox, label in zip(bboxes, labels):
        x_min, y_min, x_max, y_max = [int(c) for c in bbox]
        cv2.rectangle(image_with_bbox, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
        cv2.putText(image_with_bbox, label, (x_min, y_min - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    return image_with_bbox


def show_combined(rgb, depth, bboxes, labels):
    """Display RGB and depth images with bounding boxes."""

    rgb_vis = draw_bounding_boxes_on_rgb(rgb, bboxes, labels)

    depth_colored = cv2.applyColorMap(cv2.convertScaleAbs(depth, alpha=0.03), cv2.COLORMAP_JET)
    for bbox in bboxes:
        x_min, y_min, x_max, y_max = [int(c) for c in bbox]
        cv2.rectangle(depth_colored, (x_min, y_min), (x_max, y_max), (255, 0, 0), 2)

    h = 480
    rgb_resized = cv2.resize(rgb_vis, (int(rgb_vis.shape[1] * h / rgb_vis.shape[0]), h))
    depth_resized = cv2.resize(depth_colored, (rgb_resized.shape[1], h))

    combined = np.hstack((rgb_resized, depth_resized))
    cv2.imshow("RGB + Depth + BBoxes", combined)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        print("Quitting...")
        cv2.destroyAllWindows()
        exit(0)


def plot_bbox(image, data):
    """(Optional) Visualize results using matplotlib."""
    fig, ax = plt.subplots()
    ax.imshow(image)

    for bbox, label in zip(data['bboxes'], data['labels']):
        x1, y1, x2, y2 = bbox
        rect = patches.Rectangle(
            (x1, y1), x2-x1, y2-y1, linewidth=1, edgecolor='r', facecolor='none')
        ax.add_patch(rect)
        plt.text(x1, y1, label, color='white', fontsize=8,
                 bbox=dict(facecolor='red', alpha=0.5))

    ax.axis('off')
    plt.show()

# print(torch.cuda.is_available())

# model, processor = init()

# url = "https://disabilityarts.online/wp-content/uploads/2022/02/pulmonaryhypertension-500x375.jpg" # "https://huggingface.co/datasets/huggingface/documentation-images/resolve/main/transformers/tasks/car.jpg?download=true"
# image = Image.open(requests.get(url, stream=True).raw)#.convert("RGB")

# TASK = '<CAPTION_TO_PHRASE_GROUNDING>'
# text_input = "Give my my glasses and tissues."

# results = run_example(model, processor, TASK, text_input, image)
# print(results)

# plot_bbox(image, results['<CAPTION_TO_PHRASE_GROUNDING>'])
