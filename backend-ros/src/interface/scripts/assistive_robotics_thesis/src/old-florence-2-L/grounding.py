'''
asdf
'''

from transformers import AutoProcessor, AutoModelForCausalLM
from PIL import Image
import requests
# import copy
import torch
import time

import matplotlib.pyplot as plt
import matplotlib.patches as patches

def init():
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu") # ("cpu")
    # torch.backends.cuda.matmul.allow_tf32 = True

    torch_dtype = torch.float16 if torch.cuda.is_available() else torch.float32 # .float32

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
    start_total = time.time()

    # Build prompt
    prompt = task_prompt if text_input is None else task_prompt + text_input
    
    # Preprocessing
    start = time.time()
    inputs = processor(text=prompt, images=image, return_tensors="pt")
    inputs["input_ids"] = inputs["input_ids"].to("cuda")
    inputs["pixel_values"] = inputs["pixel_values"].to("cuda", dtype=torch.float16)
    # .to('cuda', torch.float16) # ('cpu')
    # Move tensors to GPU with float16
    # inputs = {k: v.to("cuda", dtype=torch.float16) for k, v in inputs.items()}
    print(f"Preprocessing: {time.time() - start:.2f}s")
    print(inputs["input_ids"].device)
    print(inputs["pixel_values"].device)
    print(model.device)

    print("\n--- Debug Info ---")
    print("Model ID:", id(model))
    print("Input ID shape:", inputs["input_ids"].shape)
    print("Pixel shape:", inputs["pixel_values"].shape)
    print("Prompt:", prompt)
    print("Torch memory allocated:", torch.cuda.memory_allocated() / 1e6, "MB")
    print("--- End Debug ---\n")

    # Inference
    start = time.perf_counter()

    model.eval()
    with torch.inference_mode():
        generated_ids = model.generate(
            input_ids=inputs["input_ids"], #.cuda(),
            pixel_values=inputs["pixel_values"], #.cuda(),
            # max_new_tokens=1024,
            # max_new_tokens=256,
            max_new_tokens=64,
            early_stopping=False,
            do_sample=False,
            # num_beams=3,
            num_beams=1
        )
    print(f"Inference: {time.perf_counter() - start:.2f}s")

    # Post-processing (decode + parse)
    start = time.time()
    print("Generated IDs: ", generated_ids)
    generated_text = processor.batch_decode(
        generated_ids, skip_special_tokens=False)[0]
    print("Generated text: ", generated_text)
    # parsed_answer = processor.post_process_generation(
    #     generated_text,
    #     task=task_prompt,
    #     image_size=image.size #(image.shape[1], image.shape[0]) # (image.width, image.height)
    # )
    print("📐 Image size type:", type(image.size), "value:", image.size)

    try:
        parsed_answer = processor.post_process_generation(
            generated_text,
            task=task_prompt,
            image_size=(image.shape[1], image.shape[0])
        )
    except Exception as e:
        print("❌ Error during post-processing:", e)
        print("📝 Generated text:", repr(generated_text))
        return {}, generated_text

    print("📦 Parsed answer:", parsed_answer)

    print(f"Post-processing: {time.time() - start:.2f}s")
    print(f"⏱Total: {time.time() - start_total:.2f}s")

    return parsed_answer, generated_text


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

# url = "https://disabilityarts.online/wp-content/uploads/2022/02/pulmonaryhypertension-500x375.jpg" # "https://huggingface.co/datasets/huggingface/documentation-images/resolve/main/transformers/tasks/car.jpg?download=true"
# image = Image.open(requests.get(url, stream=True).raw)#.convert("RGB")

# TASK = '<CAPTION_TO_PHRASE_GROUNDING>'
# text_input = "Give my my glasses and tissues."

# results = run_example(model, processor, TASK, text_input, image)
# print(results)

# plot_bbox(image, results['<CAPTION_TO_PHRASE_GROUNDING>'])
