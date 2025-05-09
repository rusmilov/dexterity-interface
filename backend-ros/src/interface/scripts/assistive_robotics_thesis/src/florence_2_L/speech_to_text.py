'''
File for running Azure Speech SDK's speech-to-text capabilities on personal machine.
'''

import os
from dotenv import load_dotenv
import azure.cognitiveservices.speech as speechsdk


def speak_to_microphone(api_key, region, device_name, output_file):
    '''
    Method to run when using microphone for speech-to-text.

    Args:
        api_key (string):       key for speech service
        region (string):        resource location
        device_name (string):   instance path for specific input audio device
        output_file (string):   name of file to write output to
    '''
    # kinect_device = device_name or find_kinect_device_name()
    
    # if not kinect_device:
    #     print("Azure Kinect mic not found. Falling back to default mic.")
    audio_config = speechsdk.audio.AudioConfig(use_default_microphone=True)
    # else:
    #     print(f"Using microphone: {kinect_device}")
    #     audio_config = speechsdk.audio.AudioConfig(device_name=kinect_device)

    speech_config = speechsdk.SpeechConfig(subscription=api_key, region=region)
    speech_config.speech_recognition_language = "en-US"

    # audio_config = speechsdk.audio.AudioConfig(use_default_microphone=True)

    speech_recognizer = speechsdk.SpeechRecognizer(
        speech_config=speech_config, 
        audio_config=audio_config
    )

    # set timeout durations
    speech_recognizer.properties.set_property(
        speechsdk.PropertyId.SpeechServiceConnection_InitialSilenceTimeoutMs, '60000')  # 60 seconds
    speech_recognizer.properties.set_property(
        speechsdk.PropertyId.SpeechServiceConnection_EndSilenceTimeoutMs, '20000')  # 20 seconds

    # clear the file at the beginning by opening it in write mode
    with open(output_file, "w", encoding="utf-8") as file:
        pass

    # create a loop to wait for the "start session" phrase
    print("Waiting for 'start session' to begin...")

    while True:
        # capture speech recognition result for the "start session" phrase
        speech_recognition_result = speech_recognizer.recognize_once_async().get()

        if speech_recognition_result.reason == speechsdk.ResultReason.RecognizedSpeech:
            if "start session" in speech_recognition_result.text.lower():
                print("Session started!")
                break  # exit the loop and start recognizing and writing to file

    print("Speak into your microphone. Say 'stop session' to end.")
    acc_transcription = "" # store recognized text

    with open(output_file, "a", encoding="utf-8") as file:  # open the file in append mode
        while True:
            speech_recognition_result = speech_recognizer.recognize_once_async().get()

            if speech_recognition_result.reason == speechsdk.ResultReason.RecognizedSpeech:
                print("Recognize: {}".format(speech_recognition_result.text))
                # write the recognized text to the file
                file.write(speech_recognition_result.text + "\n")
                acc_transcription += speech_recognition_result.text + " "
                if "stop session" in speech_recognition_result.text.lower():
                    print('Session end by user.')
                    break
            elif speech_recognition_result.reason == speechsdk.ResultReason.NoMatch:
                print("No speech could be recognized: {}".format(
                    speech_recognition_result.no_match_details))
            elif speech_recognition_result.reason == speechsdk.ResultReason.Canceled:
                cancellation_details = speech_recognition_result.cancellation_details
                print("Speech Recognition canceled: {}".format(
                    cancellation_details.reason))
                if cancellation_details.reason == speechsdk.CancellationReason.Error:
                    print("Error details: {}".format(
                        cancellation_details.error_details))
                    print("Did you set the speech resource key and region values?")

    return acc_transcription


# load_dotenv()

# api_key_env = os.getenv('api_key')
# region_env = os.getenv('region')
# device_name_env = os.getenv('device_name')
# OUTPUT_FILE = "transcription.txt"

# speak_to_microphone(api_key=api_key_env, region=region_env,
#                     device_name=device_name_env, output_file=OUTPUT_FILE)
