'''
asdf
'''

from transformers import AutoProcessor, AutoModelForCausalLM
from PIL import Image
import requests
# import copy
import torch

import matplotlib.pyplot as plt
import matplotlib.patches as patches

def init():
    device = torch.device("cpu") # "cuda:0" if torch.cuda.is_available() else "cpu"
    torch_dtype = torch.float32 # torch.float16 if torch.cuda.is_available() else torch.float32

    model = AutoModelForCausalLM.from_pretrained(
        "microsoft/Florence-2-large", torch_dtype=torch_dtype, trust_remote_code=True).to(device)
    processor = AutoProcessor.from_pretrained(
        "microsoft/Florence-2-large", trust_remote_code=True)

    return model, processor


def run_example(model, processor, task_prompt=None, text_input=None, image=None):
    if text_input is None:
        prompt = task_prompt
    else:
        prompt = task_prompt + text_input
    inputs = processor(text=prompt, images=image,
                       return_tensors="pt").to('cpu') # ('cuda', torch.float16)
    generated_ids = model.generate(
        input_ids=inputs["input_ids"], # .cuda(),
        pixel_values=inputs["pixel_values"], # .cuda(),
        max_new_tokens=1024,
        early_stopping=False,
        do_sample=False,
        num_beams=3,
    )
    generated_text = processor.batch_decode(
        generated_ids, skip_special_tokens=False)[0]
    parsed_answer = processor.post_process_generation(
        generated_text,
        task=task_prompt,
        image_size=(image.width, image.height)
    )

    return parsed_answer


def plot_bbox(image, data):
   # Create a figure and axes
    fig, ax = plt.subplots()

    # Display the image
    ax.imshow(image)

    # Plot each bounding box
    for bbox, label in zip(data['bboxes'], data['labels']):
        # Unpack the bounding box coordinates
        x1, y1, x2, y2 = bbox
        # Create a Rectangle patch
        rect = patches.Rectangle(
            (x1, y1), x2-x1, y2-y1, linewidth=1, edgecolor='r', facecolor='none')
        # Add the rectangle to the Axes
        ax.add_patch(rect)
        # Annotate the label
        plt.text(x1, y1, label, color='white', fontsize=8,
                 bbox=dict(facecolor='red', alpha=0.5))

    # Remove the axis ticks and labels
    ax.axis('off')

    # Show the plot
    plt.show()

# print(torch.cuda.is_available())

# model, processor = init()

# url = "https://huggingface.co/datasets/huggingface/documentation-images/resolve/main/transformers/tasks/car.jpg?download=true"
# image = Image.open(requests.get(url, stream=True).raw)

# TASK = '<CAPTION_TO_PHRASE_GROUNDING>'
# text_input = "A green car parked in front of a yellow building."

# results = run_example(model, processor, TASK, text_input)
# print(results)

# plot_bbox(image, results['<CAPTION_TO_PHRASE_GROUNDING>'])
