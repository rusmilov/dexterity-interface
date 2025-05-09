import os
import time
import queue
import threading
import numpy as np
import cv2
import torch
from PIL import Image
from dotenv import load_dotenv

from pyk4a import PyK4A, Config, DepthMode, ColorResolution, FPS, CalibrationType
from pyk4a.module import k4a_module
from .speech_to_text import speak_to_microphone
from .vision_model import init, run_example, show_combined, draw_bounding_boxes_on_rgb

# Global Constants

COLOR_RES_720P = 1  # Enum value for 1280x720
DEFAULT = "Locate the objects in frame."
DEFAULT_TASK = '<OD>'
TASK = '<CAPTION_TO_PHRASE_GROUNDING>'

# Shared Resources
latest_capture = None
capture_lock = threading.Lock()
kinect = None
show_imgs = False
latest_bboxes = []
latest_labels = []
bbox_lock = threading.Lock()


def initialize_kinect():
    """Initializes the Azure Kinect camera."""
    global kinect
    print("Initializing Kinect...")
    kinect = PyK4A(Config(
        color_resolution=ColorResolution.RES_720P,
        depth_mode=DepthMode.NFOV_2X2BINNED,
        camera_fps=FPS.FPS_15  # Reduce pressure on buffer
    ))
    kinect.start()
    print("Kinect started.")
    return kinect


def kinect_capture_thread():
    """Continuously captures frames from Kinect and adds to global buffer."""
    global latest_capture
    global kinect
    while True:
        try:
            capture = kinect.get_capture()
            with capture_lock:
                latest_capture = capture
        except Exception as e:
            print("Capture failed:", e)
        time.sleep(0.01)


def get_latest_capture():
    with capture_lock:
        return latest_capture


def align_depth_to_color(depth_image):
    """Aligns the depth image to the color image using Kinect's transformations."""
    global kinect
    transformation_handle = kinect.calibration.transformation_handle
    aligned = k4a_module.transformation_depth_image_to_color_camera(
        transformation_handle, 1, depth_image, COLOR_RES_720P
    )
    return aligned


def extract_depth_info(depth_image, bbox):
    """Extracts depth statistics for a given bounding box region."""
    x_min, y_min, x_max, y_max = [int(c) for c in bbox]
    cropped = depth_image[y_min:y_max, x_min:x_max]
    valid = cropped[cropped > 0]
    avg = valid.mean() if valid.size > 0 else 0
    mn = valid.min() if valid.size > 0 else 0
    mx = valid.max() if valid.size > 0 else 0
    return avg, mn, mx


def pixel_to_coord(depth, px, py):
    """ 
    Given a point in the image (px, py) and depth, returns the (x,y,z) coordinate in mm relative to the camera. 
    (px, py) = (0,0) is the top left corner of the picture where the x axis increases to the right and the y axis 
    increases downard.For the final returned coordinate, x is forward, y is left, and z is up, relative to the 
    center of the camera (possible relative to the center of the right camera lense).

    Source on Azure coord system: https://learn.microsoft.com/en-us/answers/questions/344156/where-is-the-origin-(0-0-0)-of-depth-camera-locate
    
    If this fails, returns None and prints the errror.
    """

    global kinect
    calibration = kinect.calibration 
    try:
        coord = calibration.convert_2d_to_3d(
            coordinates=(float(px), float(py)),
            depth=float(depth),
            source_camera=CalibrationType.COLOR  
        )
    except Exception as e:
        print(f"\033[91mError\033[0m: Conversion from 2D coordinates to 3D coordinates failed: {e}")
        return 
    
    # Convert to traditional coords where x is straight, y is right, and z is up
    # centered at camera.
    return (coord[2], -coord[0], -coord[1]) # x, y, z

# def image_display_loop():
#     while SHOW_IMAGES:
#         capture = get_latest_capture()
#         if capture and capture.color is not None:
#             frame_rgb = capture.color[:, :, :3]
#             cv2.imshow("Live Feed", frame_rgb)
#             if cv2.waitKey(30) & 0xFF == ord('q'):
#                 break
#         time.sleep(0.05)
#     cv2.destroyAllWindows()

def image_display_loop():
    global latest_bboxes, latest_labels
    while True:
        capture = get_latest_capture()
        if capture and capture.color is not None:
            frame_rgb = capture.color[:, :, :3]
            with bbox_lock:
                bboxes_copy = list(latest_bboxes)
                labels_copy = list(latest_labels)
            # Draw bounding boxes
            image_with_boxes = draw_bounding_boxes_on_rgb(frame_rgb, bboxes_copy, labels_copy)

            cv2.imshow("Live Feed", image_with_boxes)
            if cv2.waitKey(30) & 0xFF == ord('q'):
                break
        time.sleep(0.05)
    cv2.destroyAllWindows()

    

def command(model, processor, prompt=""):
    global show_imgs
    global latest_bboxes, latest_labels

    task = TASK
    if not prompt:
        task = DEFAULT_TASK
    
    capture = get_latest_capture()

    if capture.color is None or capture.depth is None:
        print("Invalid frame.")
        return

    frame_rgb = np.array(Image.fromarray(capture.color[:, :, :3]))
    depth_image_raw = capture.depth

    if depth_image_raw is None or np.all(depth_image_raw == 0):
        print("No valid depth.")
        return

    depth_image = align_depth_to_color(depth_image_raw)
    results, whole_response = run_example(model, processor, task, prompt, frame_rgb)

    bboxes = results.get(task, {}).get("bboxes", [])
    labels = results.get(task, {}).get("labels", ["" for _ in bboxes])

    objects = []
    for i, bbox in enumerate(bboxes):
        avg_d, min_d, max_d = extract_depth_info(depth_image, bbox)
        
        center_x = (bbox[0] + bbox[2]) / 2
        center_y = (bbox[1] + bbox[3]) / 2
        center_coord = pixel_to_coord(avg_d, center_x, center_y)
        if not center_coord:
            print(f"\033[91mError\033[0m: Could not calculate location of object {labels[i]}.")
        else:

            objects.append({
                'label': labels[i],
                'bounding_box': bbox,
                'depth_min': min_d,
                'depth_max': max_d,
                'center_coord': center_coord,
            })

    print(objects)
    # if show_imgs:
    #     print("IN HERE")
    #     show_combined(frame_rgb, depth_image, bboxes, labels)
    with bbox_lock:
        latest_bboxes = bboxes
        latest_labels = labels
        print("HERE!")
        
    return objects




def warmup(show_images=False):
    """ Starts the model, process, kinect and returns them. Also warms up florence."""
    global kinect
    global show_imgs
    model, processor = init()
    initialize_kinect()

    show_imgs = show_images 

    print("Model on:", next(model.parameters()).device)
    threading.Thread(target=kinect_capture_thread, args=(), daemon=True).start()

    while get_latest_capture() is None:
        time.sleep(0.05)


    # Warm up the model
    print("Warming up Florence...")
    dummy_image = Image.new("RGB", (768, 768), color=(0, 0, 0))
    inputs = processor(text="<CAPTION_TO_PHRASE_GROUNDING>test", images=dummy_image, return_tensors="pt")
    inputs["input_ids"] = inputs["input_ids"].to("cuda")
    inputs["pixel_values"] = inputs["pixel_values"].to("cuda", dtype=torch.float16)

    with torch.inference_mode():
        model.generate(
            input_ids=inputs["input_ids"],
            pixel_values=inputs["pixel_values"],
            max_new_tokens=64,
            do_sample=False,
            num_beams=1,
            early_stopping=False
        )

    # # Display blank window to trigger OpenCV GUI
    # if show_imgs:
    #     cv2.imshow("RGB + Depth + BBoxes", np.zeros((480, 1280, 3), dtype=np.uint8))
    #     cv2.waitKey(1)
    #     time.sleep(0.1)

    if show_imgs:
        threading.Thread(target=image_display_loop, daemon=True).start()
    
    return model, processor

def run_workflow():

    load_dotenv()
    api_key = os.getenv('api_key')
    region = os.getenv('region')
    device_name = os.getenv('device_name')
    output_file = "transcription.txt"
   

    model, processor = warmup()
    command(model, processor, "")



    while True:
        transcription = speak_to_microphone(
            api_key=api_key,
            region=region,
            device_name=device_name,
            output_file=output_file
        )


        command(model, processor, transcription )

        time.sleep(0.01)
    

if __name__ == "__main__":
    try:
        run_workflow()

    except Exception as e:
        print("\033[91mError\033[0m:", e)
    finally:
        if kinect:
            print("Stopping Kinect...")
            try:
                kinect.stop()
            except Exception as e:
                print("\033[91mError\033[0m while stopping Kinect:", e)
        cv2.destroyAllWindows()
