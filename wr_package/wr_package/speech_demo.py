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
        audio = recognizer.listen(source, phrase_time_limit=15)
    return audio

def parse_command(text):
    text = re.sub(r'°', ' degrees', text)
    text = re.sub(r'\b(m|M)\b', 'meters', text)
    # Define command words and units
    commands = ["forward", "forwards", "backward", "backwards", "left", "right", "stop", "come"]
    units = ["meters", 'degrees']

     # Regular expression pattern to match whole words for commands, units, and numbers
    pattern = (
          r'\b\d+\b'  # Match whole numbers
        + r'|'
        + r'\b(?:' + '|'.join(re.escape(command) for command in commands) + r')\b'  # Match whole words for commands
        + r'|'
        + r'(?:' + '|'.join(re.escape(unit) for unit in units) + r')'  # Match units
    )

    # Find matches and add them to the list
    extracted_words = []
    for match in re.findall(pattern, text):
        if '°' in match:  # If degree symbol is found
            number = match.replace('°', '')  # Remove degree symbol
            extracted_words.append(number)
            extracted_words.append('degrees')
        else:
            extracted_words.append(match)
    
    print(f"Extracted sequence: {extracted_words}")
    return extracted_words

    

def main():
    recognizer = sr.Recognizer()
    recognizer.pause_threshold = 1.5
    microphone = sr.Microphone()

    try:
        while True:
            print("Say 'Hey Robot' to start interaction.")
            audio = listen_for_speech(recognizer, microphone)

            # Recognizing the initial keyword
            try:
                text = recognizer.recognize_google(audio)
                print(f"Recognized: {text}")

                if "hey robot" in text.lower():
                    speak("Yes I am listening")
                    print("Listening for your command...")

                    # Wait for a full command
                    while True:
                        command_audio = listen_for_speech(recognizer, microphone)
                        command_text = recognizer.recognize_google(command_audio)
                        
                        
                        print(f"Command: {command_text}")
                        if command_text.strip():  # If some speech was detected
                            parse_command(command_text.lower())  # Parse the command
                            time.sleep(3)  # Wait for 3 seconds to see if speech is complete
                            speak("Executing the commands")
                            break

            except sr.UnknownValueError:
                print("Sorry, I did not understand that.")
            except sr.RequestError:
                print("Sorry, there was an error with the Google Speech Recognition service.")

    except KeyboardInterrupt:
        print("Program stopped manually.")
        return

if __name__ == "__main__":
    main()