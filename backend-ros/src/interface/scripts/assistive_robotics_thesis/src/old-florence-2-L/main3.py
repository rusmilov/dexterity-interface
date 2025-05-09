import os
import time
import queue
import threading
import numpy as np
import cv2
import torch
from PIL import Image
from dotenv import load_dotenv

from pyk4a import PyK4A, Config, DepthMode, ColorResolution
from pyk4a.module import k4a_module
from speech_to_text import speak_to_microphone
from grounding import init, run_example

SHOW_IMAGES = True
capture_queue = queue.Queue(maxsize=1)
kinect = None  # Global access for transformation

COLOR_RES_720P = 1  # Enum value for 1280x720


def initialize_kinect():
    global kinect
    print("Initializing Kinect...")
    kinect = PyK4A(Config(
        color_resolution=ColorResolution.RES_720P,
        depth_mode=DepthMode.NFOV_2X2BINNED,
    ))
    kinect.start()
    print("Kinect started.")
    return kinect


def kinect_capture_thread(k):
    while True:
        if not capture_queue.full():
            capture = k.get_capture()
            capture_queue.put(capture)
        time.sleep(0.05)


def flush_capture_queue():
    while not capture_queue.empty():
        try:
            capture_queue.get_nowait()
        except queue.Empty:
            break


def align_depth_to_color(depth_image):
    transformation_handle = kinect.calibration.transformation_handle
    aligned = k4a_module.transformation_depth_image_to_color_camera(
        transformation_handle, 1, depth_image, COLOR_RES_720P
    )
    return aligned


def extract_depth_info(depth_image, bbox):
    x_min, y_min, x_max, y_max = [int(c) for c in bbox]
    cropped = depth_image[y_min:y_max, x_min:x_max]
    valid = cropped[cropped > 0]
    avg = valid.mean() if valid.size > 0 else 0
    mn = valid.min() if valid.size > 0 else 0
    mx = valid.max() if valid.size > 0 else 0
    return avg, mn, mx


def draw_bounding_boxes_on_rgb(image, bboxes, labels):
    image_with_bbox = image.copy()
    for bbox, label in zip(bboxes, labels):
        x_min, y_min, x_max, y_max = [int(c) for c in bbox]
        cv2.rectangle(image_with_bbox, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
        cv2.putText(image_with_bbox, label, (x_min, y_min - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    return image_with_bbox


def show_combined(rgb, depth, bboxes, labels):
    # if rgb is None or depth is None or rgb.size == 0 or depth.size == 0:
    #     print("⚠️ Skipping display: RGB or depth image is empty.")
    #     return

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


def default_command(model, processor, task_prompt):
    # flush_capture_queue()
    if capture_queue.empty():
        print("No frame in queue.")
        return

    capture = capture_queue.get()
    if capture.color is None or capture.depth is None:
        print("Invalid frame.")
        return

    frame_rgb = np.array(Image.fromarray(capture.color[:, :, :3]))
    depth_image_raw = capture.depth

    if depth_image_raw is None or np.all(depth_image_raw == 0):
        print("No valid depth.")
        return

    depth_image = align_depth_to_color(depth_image_raw)

    print(f"Depth shape: {depth_image.shape} | min={depth_image.min()} max={depth_image.max()}")
    start = time.time()
    results, whole_response = run_example(model, processor, DEFAULT_TASK, "", frame_rgb)
    print("🤖", whole_response)
    print(f"Inference time: {time.time() - start:.2f} seconds")
    bboxes = results.get(DEFAULT_TASK, {}).get("bboxes", [])
    labels = results.get(DEFAULT_TASK, {}).get("labels", ["" for _ in bboxes])

    for bbox in bboxes:
        _, min_d, max_d = extract_depth_info(depth_image, bbox)
        print(f"BBox: {bbox} → Min: {min_d} mm | Max: {max_d} mm")

    if SHOW_IMAGES:
        show_combined(frame_rgb, depth_image, bboxes, labels)

# def default_command(model, processor, task_prompt):
#     if capture_queue.empty():
#         print("No frame in queue.")
#         return

#     t0 = time.time()
#     capture = capture_queue.get()
#     t1 = time.time()

#     if capture.color is None or capture.depth is None:
#         print("Invalid frame.")
#         return

#     # Image conversion
#     pil_img = Image.fromarray(capture.color[:, :, :3])
#     t2 = time.time()

#     depth_image_raw = capture.depth
#     if depth_image_raw is None or np.all(depth_image_raw == 0):
#         print("No valid depth.")
#         return

#     depth_image = align_depth_to_color(depth_image_raw)
#     t3 = time.time()

#     results, whole_response = run_example(model, processor, task_prompt, "", pil_img)
#     t4 = time.time()

#     print("🤖", whole_response)

#     print(f"⏱ Timing breakdown:")
#     print(f"📸 Frame get:        {t1 - t0:.2f}s")
#     print(f"🖼  Convert to PIL:  {t2 - t1:.2f}s")
#     print(f"📏 Align depth:     {t3 - t2:.2f}s")
#     print(f"🤖 Model inference: {t4 - t3:.2f}s")
#     print(f"🟢 Total:           {t4 - t0:.2f}s")


def run_workflow(model, processor, task_prompt, k):
    load_dotenv()
    api_key = os.getenv('api_key')
    region = os.getenv('region')
    device_name = os.getenv('device_name')
    output_file = "transcription.txt"

    print("Model on:", next(model.parameters()).device)
    threading.Thread(target=kinect_capture_thread, args=(k,), daemon=True).start()

    # print("Warming up...")
    while capture_queue.empty():
        time.sleep(0.05)
    # print("Ready.")

    # 🚀 Warm up the model
    t0 = time.time()
    print("Warming up Florence...")
    dummy_image = Image.new("RGB", (768, 768), color=(0, 0, 0))
    inputs = processor(text="<CAPTION_TO_PHRASE_GROUNDING>test", images=dummy_image, return_tensors="pt")
    inputs["input_ids"] = inputs["input_ids"].to("cuda")
    inputs["pixel_values"] = inputs["pixel_values"].to("cuda", dtype=torch.float16)

    print("🧠 Inside run_example()")
    print("Model ID:", id(model))
    print("Device:", next(model.parameters()).device)
    print("Pixel shape:", inputs["pixel_values"].shape)
    print("Allocated mem:", torch.cuda.memory_allocated() / 1e6, "MB")

    with torch.inference_mode():
        model.generate(
            input_ids=inputs["input_ids"],
            pixel_values=inputs["pixel_values"],
            max_new_tokens=64,
            do_sample=False,
            num_beams=1,
            early_stopping=False
        )
    print("Warm-up complete.")
    t1 = time.time()
    print(f"🤖 Model inference: {t1 - t0:.2f}s")
    cv2.imshow("RGB + Depth + BBoxes", np.zeros((480, 1280, 3), dtype=np.uint8))
    cv2.waitKey(1)
    time.sleep(0.1)
    default_command(model, processor, task_prompt)

    flush_capture_queue()

    while capture_queue.empty():
        time.sleep(0.05)

    while True:
        transcription = speak_to_microphone(
            api_key=api_key,
            region=region,
            device_name=device_name,
            output_file=output_file
        )

        if capture_queue.empty():
            print("No frame in queue.")
            continue

        flush_capture_queue()
        
        capture = capture_queue.get()
        if capture.color is None or capture.depth is None:
            print("Invalid frame.")
            continue

        frame_rgb = np.array(Image.fromarray(capture.color[:, :, :3]))
        depth_image_raw = capture.depth

        if depth_image_raw is None or np.all(depth_image_raw == 0):
            print("No valid depth.")
            continue

        depth_image = align_depth_to_color(depth_image_raw)
        print(f"Depth shape: {depth_image.shape} | min={depth_image.min()} max={depth_image.max()}")

        results, whole_response = run_example(model, processor, task_prompt, transcription, frame_rgb)
        print("🤖", whole_response)
        print("🤖", whole_response)
        bboxes = results.get('<CAPTION_TO_PHRASE_GROUNDING>', {}).get("bboxes", [])
        labels = results.get('<CAPTION_TO_PHRASE_GROUNDING>', {}).get("labels", ["" for _ in bboxes])

        print("🧪 BBoxes:", bboxes)
        for bbox in bboxes:
            _, min_d, max_d = extract_depth_info(depth_image, bbox)
            print(f"BBox: {bbox} → Min: {min_d} mm | Max: {max_d} mm")

        if SHOW_IMAGES:
            show_combined(frame_rgb, depth_image, bboxes, labels)

        time.sleep(0.01)


DEFAULT = "Locate the objects in frame."
DEFAULT_TASK = '<OD>'
TASK = '<CAPTION_TO_PHRASE_GROUNDING>'

if __name__ == "__main__":
    try:
        model, processor = init()
        kinect = initialize_kinect()
        run_workflow(model, processor, TASK, kinect)
    except Exception as e:
        print("Error:", e)
    finally:
        if kinect:
            print("Stopping Kinect...")
            try:
                kinect.stop()
            except Exception as e:
                print("Error while stopping Kinect:", e)
        cv2.destroyAllWindows()
