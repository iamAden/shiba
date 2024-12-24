import speech_recognition as sr
import socket

def main():
    recognizer = sr.Recognizer()
    
    try:
        mic = sr.Microphone()
    except OSError as e:
        print(f"Microphone initialization failed: {e}")
        return

    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect(("127.0.0.1", 12345))  # Connect to the Webots controller
    except ConnectionError as e:
        print(f"Socket connection failed: {e}")
        return

    print("Ready to receive commands...")
    try:
        while True:
            with mic as source:
                recognizer.adjust_for_ambient_noise(source)
                print("Listening...")
                audio = recognizer.listen(source)

                try:
                    command = recognizer.recognize_google(audio)
                    print(f"Recognized command: {command}")
                    s.sendall(command.encode('utf-8'))  # Send command to Webots
                except sr.UnknownValueError:
                    print("Could not understand the command. Please try again.")
                except sr.RequestError as e:
                    print(f"API error: {e}")
    except KeyboardInterrupt:
        print("\nTerminating the program.")
    finally:
        s.close()

if __name__ == "__main__":
    main()

