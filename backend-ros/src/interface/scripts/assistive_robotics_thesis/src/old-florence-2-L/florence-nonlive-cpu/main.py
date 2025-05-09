'''
asdf
'''

import os
from dotenv import load_dotenv
from PIL import Image
import requests
from speech_to_text import speak_to_microphone
from grounding import init, run_example, plot_bbox

# Full Workflow: Run Speech-to-Text and Phrase Grounding
def run_workflow(image_url, task_prompt):
    '''
    asdf
    '''
    load_dotenv()

    api_key_env = os.getenv('api_key')
    region_env = os.getenv('region')
    device_name_env = os.getenv('device_name')
    output_file = "transcription.txt"

    # Step 1: Wait for "start session" and transcribe Audio to Text
    transcription = speak_to_microphone(api_key=api_key_env, region=region_env, 
                                        device_name=device_name_env, output_file=output_file)
    print(f"Recognized Text: {transcription}")

    # Step 2: Initialize Florence Model and Processor
    model, processor = init()

    # Step 3: Load the Image
    image = Image.open(requests.get(image_url, stream=True).raw)
    
    # Step 4: Run Phrase Grounding
    results = run_example(model, processor, task_prompt, transcription, image)
    print(f"Results: {results}")

    # Step 5: Visualize the Bounding Boxes
    plot_bbox(image, results['<CAPTION_TO_PHRASE_GROUNDING>'])

# Run the Workflow
URL = "https://disabilityarts.online/wp-content/uploads/2022/02/pulmonaryhypertension-500x375.jpg" 
    # "https://huggingface.co/datasets/huggingface/documentation-images/
    # resolve/main/transformers/tasks/car.jpg?download=true"
TASK = '<CAPTION_TO_PHRASE_GROUNDING>'

run_workflow(URL, TASK)
