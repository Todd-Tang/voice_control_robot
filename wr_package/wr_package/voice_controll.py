import rclpy
from rclpy.node import Node
from custom_interfaces.msg import VoiceCommand
import speech_recognition as sr
import time
from gtts import gTTS
import playsound
import os
import re

def speak(text):
    tts = gTTS(text=text, lang="en")
    filename = "voice.mp3"
    tts.save(filename)
    playsound.playsound(filename)
    os.remove(filename)

def listen_for_speech(recognizer, microphone):
    with microphone as source:
        print("Listening...")
        recognizer.adjust_for_ambient_noise(source)
        audio = recognizer.listen(source, phrase_time_limit=7)
    return audio

class VoiceCommandPublisher(Node):
    def __init__(self):
        super().__init__('voice_command_publisher')
        self.publisher_ = self.create_publisher(VoiceCommand, '/voice_command', 10)
        self.recognizer = sr.Recognizer()
        self.recognizer.pause_threshold = 1.5
        self.microphone = sr.Microphone()

    def publish_command(self, command, number, unit):
        msg = VoiceCommand()
        msg.command = command
        msg.number = number
        msg.unit = unit
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{command}" {number} {unit}')

    def parse_and_publish_command(self, text):
        text = re.sub(r'°', ' degrees', text)
        text = re.sub(r'\b(m|M)\b', 'meters', text)
        # Define command words and units
        commands = ["up", "down", "left", "right", "stop", "come"]
        units = ["meters", 'degrees']

        pattern = (
              r'\b\d+\b'  # Match whole numbers
            + r'|'
            + r'\b(?:' + '|'.join(re.escape(command) for command in commands) + r')\b'  # Match commands
            + r'|'
            + r'(?:' + '|'.join(re.escape(unit) for unit in units) + r')'  # Match units
        )

        extracted_words = re.findall(pattern, text)
        print(f"Extracted sequence: {extracted_words}")

        if len(extracted_words) >= 3:
            command = extracted_words[0]
            number = float(extracted_words[1])
            unit = extracted_words[2]
            self.publish_command(command, number, unit)

def main(args=None):
    rclpy.init(args=args)
    voice_command_publisher = VoiceCommandPublisher()

    try:
        while True:
            # Outer loop for continuously listening for "Hey Robot"
            while True:
                print("Say 'Hey Google' to start interaction.")
                audio = listen_for_speech(voice_command_publisher.recognizer, voice_command_publisher.microphone)

                try:
                    text = voice_command_publisher.recognizer.recognize_google(audio)
                    print(f"Recognized: {text}")

                    if "hey google" in text.lower():
                        speak("Yes I am listening")
                        print("Listening for your command...")
                        break
                    else:
                        print("Waiting for 'Hey Google'...")

                except sr.UnknownValueError:
                    print("Sorry, I did not understand that.")
                except sr.RequestError:
                    print("Sorry, there was an error with the Google Speech Recognition service.")

            # Inner loop for listening to a command after "Hey Google" is recognized
            while True:
                command_audio = listen_for_speech(voice_command_publisher.recognizer, voice_command_publisher.microphone)
                try:
                    command_text = voice_command_publisher.recognizer.recognize_google(command_audio)
                    print(f"Command: {command_text}")

                    if command_text.strip():  # If some speech was detected
                        voice_command_publisher.parse_and_publish_command(command_text.lower())  # Parse and publish the command
                        time.sleep(3)  # Wait for 3 seconds to see if speech is complete
                        speak("Executing the commands")
                        break  # Exit the inner loop after executing the command

                except sr.UnknownValueError:
                    print("Sorry, I did not understand that. Please repeat your command.")
                except sr.RequestError:
                    print("Sorry, there was an error with the Google Speech Recognition service.")

            rclpy.spin_once(voice_command_publisher)

    except KeyboardInterrupt:
        print("Program stopped manually.")

    rclpy.shutdown()

if __name__ == "__main__":
    main()