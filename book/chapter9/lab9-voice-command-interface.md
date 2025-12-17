# Chapter 9 Lab: Voice Command Interface for Human-Robot Interaction

## Objective

The goal of this lab is to implement a voice command interface for human-robot interaction, enabling natural communication between humans and robots through speech recognition and natural language processing. You will learn to process voice commands, interpret user intent, and generate appropriate robot responses.

## Learning Outcomes

By the end of this lab, you should be able to:

- Implement speech recognition for robot command processing
- Design natural language understanding for robot commands
- Create speech synthesis for robot responses
- Integrate voice interface with robot control systems
- Handle multi-turn conversations with robots
- Evaluate voice interface usability and performance

## Prerequisites

- ROS2 Humble Hawksbill installed
- Python programming skills
- Basic understanding of natural language processing
- Microphone and speaker for audio I/O

## Materials Needed

- Computer with ROS2 installed and audio capabilities
- Microphone for speech input
- Speaker for speech output
- Text editor or IDE
- Terminal access

## Background

Voice interfaces are crucial for natural human-robot interaction, allowing users to communicate with robots using natural language. This lab explores speech recognition, natural language understanding, and speech synthesis to create an intuitive voice command interface.

## Lab Procedure

### Part 1: Setting Up the Environment (15 minutes)

1. **Create a new ROS2 package for this lab:**
   ```bash
   cd ~/ros2_labs/src
   ros2 pkg create --build-type ament_python lab_voice_interface --dependencies rclpy std_msgs geometry_msgs sensor_msgs sound_msgs
   ```

2. **Navigate to the package directory:**
   ```bash
   cd lab_voice_interface
   ```

3. **Create the main script directory:**
   ```bash
   mkdir -p lab_voice_interface
   ```

### Part 2: Installing Required Libraries (10 minutes)

1. **Install required Python packages:**
   ```bash
   pip3 install speechrecognition pyttsx3 vosk pyaudio numpy
   ```

2. **For Vosk (offline speech recognition), install:**
   ```bash
   pip3 install vosk
   ```

### Part 3: Implementing Speech Recognition (45 minutes)

Create a speech recognition module for processing voice commands:

```python
#!/usr/bin/env python3
import speech_recognition as sr
import pyaudio
import numpy as np
import threading
import queue
import time

class SpeechRecognizer:
    def __init__(self, use_offline=True):
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        # Adjust for ambient noise
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)

        # Recognition parameters
        self.energy_threshold = 300  # Minimum audio energy to consider for recording
        self.dynamic_energy_threshold = True
        self.pause_threshold = 0.8  # Seconds of non-speaking audio before a phrase is considered complete

        # Audio processing
        self.audio_queue = queue.Queue()
        self.recording = False
        self.use_offline = use_offline

        # Offline recognition (Vosk)
        if use_offline:
            try:
                from vosk import Model
                # You'll need to download a Vosk model
                # Example: https://alphacephei.com/vosk/models
                # self.vosk_model = Model("path/to/vosk-model")
                self.use_vosk = False  # Disable if model not available
            except ImportError:
                self.use_vosk = False

    def start_listening(self, callback=None):
        """Start continuous listening for speech"""
        self.recording = True
        self.listener_thread = threading.Thread(
            target=self._continuous_listen,
            args=(callback,)
        )
        self.listener_thread.daemon = True
        self.listener_thread.start()

    def stop_listening(self):
        """Stop listening for speech"""
        self.recording = False
        if hasattr(self, 'listener_thread'):
            self.listener_thread.join(timeout=1.0)

    def _continuous_listen(self, callback):
        """Continuously listen for audio and recognize speech"""
        while self.recording:
            try:
                with self.microphone as source:
                    # Listen for audio
                    audio = self.recognizer.listen(
                        source,
                        timeout=1.0,
                        phrase_time_limit=5.0
                    )

                # Recognize speech
                text = self._recognize_speech(audio)

                if text and callback:
                    callback(text)

            except sr.WaitTimeoutError:
                # No speech detected, continue listening
                continue
            except sr.UnknownValueError:
                # Speech not understood, continue
                continue
            except sr.RequestError as e:
                print(f"Speech recognition error: {e}")
                time.sleep(1)  # Brief pause before retrying
            except Exception as e:
                print(f"Error in speech recognition: {e}")
                time.sleep(1)

    def _recognize_speech(self, audio):
        """Recognize speech from audio data"""
        try:
            if self.use_vosk:
                # Use Vosk for offline recognition
                # This is a placeholder - implement with actual Vosk model
                return self._recognize_with_vosk(audio)
            else:
                # Use online recognition (Google by default)
                text = self.recognizer.recognize_google(audio)
                return text.lower().strip()
        except:
            return None

    def _recognize_with_vosk(self, audio):
        """Offline speech recognition using Vosk"""
        # Placeholder implementation
        # In practice, you would use the Vosk API
        try:
            # Convert audio to raw data
            raw_data = audio.get_raw_data()
            # Process with Vosk model
            # result = self.vosk_model.recognize(raw_data)
            # return result.get('text', '').lower().strip()
            return None  # Placeholder
        except:
            return None

    def listen_once(self):
        """Listen for a single phrase and return the recognized text"""
        try:
            with self.microphone as source:
                print("Listening...")
                audio = self.recognizer.listen(source, timeout=5.0, phrase_time_limit=10.0)

            text = self._recognize_speech(audio)
            return text
        except sr.WaitTimeoutError:
            print("No speech detected")
            return None
        except sr.UnknownValueError:
            print("Could not understand audio")
            return None
        except sr.RequestError as e:
            print(f"Recognition error: {e}")
            return None

# Example usage and testing
if __name__ == "__main__":
    recognizer = SpeechRecognizer()

    def handle_recognized_text(text):
        print(f"Recognized: {text}")

    recognizer.start_listening(handle_recognized_text)

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        recognizer.stop_listening()
```

### Part 4: Implementing Natural Language Understanding (50 minutes)

Create a natural language understanding module to interpret commands:

```python
#!/usr/bin/env python3
import re
import json
from typing import Dict, List, Tuple, Optional

class NaturalLanguageUnderstanding:
    def __init__(self):
        # Define command patterns and their corresponding actions
        self.command_patterns = {
            # Movement commands
            'move_forward': [
                r'move forward',
                r'go forward',
                r'forward',
                r'go straight',
                r'straight',
                r'advance',
                r'proceed'
            ],
            'move_backward': [
                r'move backward',
                r'go backward',
                r'backward',
                r'back',
                r'reverse',
                r'retreat'
            ],
            'turn_left': [
                r'turn left',
                r'left',
                r'rotate left',
                r'pivot left',
                r'go left'
            ],
            'turn_right': [
                r'turn right',
                r'right',
                r'rotate right',
                r'pivot right',
                r'go right'
            ],
            'stop': [
                r'stop',
                r'hold',
                r'pause',
                r'freeze',
                r'wait'
            ],
            'wave': [
                r'wave',
                r'wave hello',
                r'greet',
                r'hello'
            ],
            'dance': [
                r'dance',
                r'do a dance',
                r'boogie',
                r'party'
            ],
            'follow_me': [
                r'follow me',
                r'follow',
                r'come with me',
                r'come on'
            ],
            'find_person': [
                r'find person',
                r'look for person',
                r'search for person',
                r'find someone'
            ],
            'bring_object': [
                r'bring me (.+)',
                r'get me (.+)',
                r'fetch (.+)',
                r'bring (.+)'
            ]
        }

        # Define object names that robot can recognize
        self.known_objects = [
            'water', 'bottle', 'cup', 'book', 'phone', 'keys',
            'apple', 'orange', 'toy', 'ball', 'pen', 'notebook'
        ]

        # Define location names
        self.known_locations = [
            'kitchen', 'living room', 'bedroom', 'office',
            'bathroom', 'dining room', 'hallway', 'garden'
        ]

        # Define person descriptors
        self.person_descriptors = [
            'person', 'man', 'woman', 'child', 'elderly',
            'young', 'tall', 'short', 'wearing red', 'wearing blue'
        ]

    def parse_command(self, text: str) -> Dict:
        """
        Parse a voice command and extract intent and parameters
        """
        text = text.lower().strip()

        # Check for each command pattern
        for action, patterns in self.command_patterns.items():
            for pattern in patterns:
                # Handle patterns with capture groups
                match = re.search(pattern, text)
                if match:
                    result = {
                        'action': action,
                        'confidence': 0.9,  # High confidence for exact matches
                        'parameters': {}
                    }

                    # Extract captured groups (for commands like 'bring me water')
                    if match.groups():
                        result['parameters']['object'] = match.group(1).strip()

                    return result

        # If no exact pattern matches, try semantic understanding
        return self._semantic_parse(text)

    def _semantic_parse(self, text: str) -> Dict:
        """
        Use semantic understanding for commands that don't match patterns exactly
        """
        words = text.split()

        # Look for keywords that indicate movement
        movement_keywords = {
            'forward': 'move_forward',
            'backward': 'move_backward',
            'back': 'move_backward',
            'left': 'turn_left',
            'right': 'turn_right',
            'stop': 'stop',
            'halt': 'stop'
        }

        for word in words:
            if word in movement_keywords:
                return {
                    'action': movement_keywords[word],
                    'confidence': 0.7,
                    'parameters': {}
                }

        # Look for object-related commands
        for obj in self.known_objects:
            if obj in text:
                if any(word in text for word in ['bring', 'get', 'fetch']):
                    return {
                        'action': 'bring_object',
                        'confidence': 0.6,
                        'parameters': {'object': obj}
                    }

        # Look for location-related commands
        for location in self.known_locations:
            if location in text:
                if any(word in text for word in ['go', 'move', 'navigate', 'head']):
                    return {
                        'action': 'navigate_to',
                        'confidence': 0.6,
                        'parameters': {'location': location}
                    }

        # If no specific action is identified, return unknown
        return {
            'action': 'unknown',
            'confidence': 0.0,
            'parameters': {'raw_text': text}
        }

    def extract_entities(self, text: str) -> Dict[str, List[str]]:
        """
        Extract named entities from text (objects, locations, people)
        """
        entities = {
            'objects': [],
            'locations': [],
            'people': []
        }

        # Extract known objects
        for obj in self.known_objects:
            if obj in text:
                entities['objects'].append(obj)

        # Extract known locations
        for location in self.known_locations:
            if location in text:
                entities['locations'].append(location)

        # Extract person descriptors
        for person_desc in self.person_descriptors:
            if person_desc in text:
                entities['people'].append(person_desc)

        return entities

    def validate_command(self, parsed_command: Dict) -> Tuple[bool, str]:
        """
        Validate if the parsed command is executable
        """
        action = parsed_command['action']
        params = parsed_command['parameters']

        # Validate specific actions
        if action == 'bring_object':
            if 'object' not in params:
                return False, "Object to bring is not specified"

            obj = params['object']
            if obj not in self.known_objects:
                return False, f"I don't know how to bring '{obj}'. I can bring: {', '.join(self.known_objects)}"

        elif action == 'navigate_to':
            if 'location' not in params:
                return False, "Destination location is not specified"

        return True, "Command is valid"

    def generate_confirmation(self, parsed_command: Dict) -> str:
        """
        Generate a confirmation message for the parsed command
        """
        action = parsed_command['action']
        params = parsed_command['parameters']

        if action == 'move_forward':
            return "Moving forward."
        elif action == 'move_backward':
            return "Moving backward."
        elif action == 'turn_left':
            return "Turning left."
        elif action == 'turn_right':
            return "Turning right."
        elif action == 'stop':
            return "Stopping."
        elif action == 'bring_object':
            obj = params.get('object', 'something')
            return f"I will bring you the {obj}."
        elif action == 'navigate_to':
            location = params.get('location', 'a location')
            return f"Going to the {location}."
        elif action == 'find_person':
            return "Looking for a person."
        elif action == 'wave':
            return "Waving hello."
        else:
            return f"Received command: {action}."

    def get_available_commands(self) -> List[str]:
        """
        Return a list of available commands
        """
        commands = []
        for action, patterns in self.command_patterns.items():
            commands.append(f"Action: {action}, Examples: {patterns[:2]}")  # Show first 2 examples
        return commands

# Example usage
if __name__ == "__main__":
    nlu = NaturalLanguageUnderstanding()

    test_commands = [
        "move forward",
        "turn left",
        "bring me water",
        "find person",
        "go to kitchen"
    ]

    for cmd in test_commands:
        parsed = nlu.parse_command(cmd)
        print(f"Command: '{cmd}' -> {parsed}")

        is_valid, message = nlu.validate_command(parsed)
        print(f"Valid: {is_valid}, Message: {message}")
        print(f"Confirmation: {nlu.generate_confirmation(parsed)}")
        print("---")
```

### Part 5: Implementing Speech Synthesis (30 minutes)

Create a speech synthesis module for robot responses:

```python
#!/usr/bin/env python3
import pyttsx3
import threading
import queue
import time

class SpeechSynthesizer:
    def __init__(self, rate=200, volume=0.9):
        self.engine = pyttsx3.init()

        # Configure speech properties
        self.engine.setProperty('rate', rate)  # Speed of speech
        self.engine.setProperty('volume', volume)  # Volume level (0.0 to 1.0)

        # Get available voices
        self.voices = self.engine.getProperty('voices')

        # Set a default voice (preferably a clear female voice if available)
        for voice in self.voices:
            if 'female' in voice.name.lower() or 'zira' in voice.name.lower():
                self.engine.setProperty('voice', voice.id)
                break

        # Queuing system for speech
        self.speech_queue = queue.Queue()
        self.speaking = False
        self.speech_thread = None

    def set_voice_properties(self, rate=None, volume=None, voice_index=None):
        """Adjust voice properties"""
        if rate is not None:
            self.engine.setProperty('rate', rate)
        if volume is not None:
            self.engine.setProperty('volume', volume)
        if voice_index is not None and 0 <= voice_index < len(self.voices):
            self.engine.setProperty('voice', self.voices[voice_index].id)

    def speak(self, text, blocking=False):
        """Speak the given text"""
        if blocking:
            self._speak_immediately(text)
        else:
            # Add to queue for non-blocking speech
            self.speech_queue.put(('speak', text))
            if not self.speaking:
                self._start_speech_thread()

    def _speak_immediately(self, text):
        """Speak text immediately (blocking)"""
        try:
            self.engine.say(text)
            self.engine.runAndWait()
        except:
            print(f"Could not speak: {text}")

    def _start_speech_thread(self):
        """Start the speech thread if not already running"""
        if self.speech_thread is None or not self.speech_thread.is_alive():
            self.speaking = True
            self.speech_thread = threading.Thread(target=self._speech_worker)
            self.speech_thread.daemon = True
            self.speech_thread.start()

    def _speech_worker(self):
        """Worker thread for handling speech queue"""
        while self.speaking:
            try:
                command, text = self.speech_queue.get(timeout=0.1)

                if command == 'speak':
                    self._speak_immediately(text)
                elif command == 'stop':
                    self.engine.stop()
                    break

            except queue.Empty:
                continue
            except Exception as e:
                print(f"Speech synthesis error: {e}")
                continue

    def stop_speaking(self):
        """Stop current speech"""
        self.speech_queue.put(('stop', None))
        self.speaking = False

    def is_speaking(self):
        """Check if the system is currently speaking"""
        return self.engine.isBusy()

    def preload_phrases(self, phrases):
        """Preload common phrases for faster response"""
        # This is a placeholder - in a real system, you might pre-synthesize common phrases
        pass

    def speak_with_emotion(self, text, emotion='neutral'):
        """Speak with different emotional tones (simulated)"""
        if emotion == 'happy':
            self.set_voice_properties(rate=220)
            self.speak(text)
            self.set_voice_properties(rate=200)  # Reset to default
        elif emotion == 'sad':
            self.set_voice_properties(rate=150, volume=0.7)
            self.speak(text)
            self.set_voice_properties(rate=200, volume=0.9)  # Reset to default
        else:
            self.speak(text)

    def speak_dialogue(self, text, is_question=False):
        """Speak dialogue with appropriate intonation"""
        if is_question:
            # Add a slight pause and rising intonation simulation
            self.speak(text + "    ")  # Extra space to simulate pause
        else:
            self.speak(text)

# Example usage
if __name__ == "__main__":
    synthesizer = SpeechSynthesizer()

    # Test different types of speech
    synthesizer.speak("Hello, I am your robot assistant.")
    time.sleep(1)
    synthesizer.speak("How can I help you today?", is_question=True)
    time.sleep(2)
    synthesizer.speak("I can move, fetch objects, and navigate to different locations.")

    # Test emotion simulation
    print("Speaking with happy emotion...")
    synthesizer.speak_with_emotion("Great! I'm happy to help you!", 'happy')

    # Keep the program running to hear the speech
    time.sleep(5)
```

### Part 6: Creating the Main Voice Interface Node (40 minutes)

Create the main ROS2 node that integrates all voice interface components:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from lab_voice_interface.speech_recognizer import SpeechRecognizer
from lab_voice_interface.nlu import NaturalLanguageUnderstanding
from lab_voice_interface.speech_synthesizer import SpeechSynthesizer

class VoiceInterfaceNode(Node):
    def __init__(self):
        super().__init__('voice_interface_node')

        # Initialize voice components
        self.speech_recognizer = SpeechRecognizer(use_offline=True)
        self.nlu = NaturalLanguageUnderstanding()
        self.speech_synthesizer = SpeechSynthesizer(rate=180, volume=0.8)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.response_pub = self.create_publisher(String, 'voice_response', 10)
        self.command_pub = self.create_publisher(String, 'parsed_command', 10)

        # Subscribers
        self.command_sub = self.create_subscription(
            String, 'voice_command', self.voice_command_callback, 10)

        # Robot state
        self.robot_state = 'idle'  # idle, moving, processing, etc.
        self.current_task = None
        self.is_listening = False

        # Voice interface parameters
        self.response_timeout = 30.0  # seconds to wait for response before timing out
        self.confirm_commands = True  # Whether to confirm received commands

        # Start listening for voice commands
        self.start_voice_listening()

        self.get_logger().info('Voice interface node initialized')

    def start_voice_listening(self):
        """Start listening for voice commands"""
        self.speech_recognizer.start_listening(self.process_recognized_text)
        self.is_listening = True
        self.get_logger().info('Voice listening started')

    def stop_voice_listening(self):
        """Stop listening for voice commands"""
        self.speech_recognizer.stop_listening()
        self.is_listening = False
        self.get_logger().info('Voice listening stopped')

    def process_recognized_text(self, text):
        """Process recognized text from speech"""
        self.get_logger().info(f'Received voice command: {text}')

        # Parse the command using NLU
        parsed_command = self.nlu.parse_command(text)

        # Validate the command
        is_valid, validation_message = self.nlu.validate_command(parsed_command)

        if is_valid:
            # Generate confirmation
            confirmation = self.nlu.generate_confirmation(parsed_command)
            self.get_logger().info(f'Command valid: {confirmation}')

            # Publish the parsed command
            cmd_msg = String()
            cmd_msg.data = str(parsed_command)
            self.command_pub.publish(cmd_msg)

            # Execute the command
            self.execute_command(parsed_command)

            # Respond to user
            if self.confirm_commands:
                self.speech_synthesizer.speak(confirmation)
        else:
            # Invalid command - provide feedback
            self.get_logger().warn(f'Invalid command: {validation_message}')
            self.speech_synthesizer.speak(f"I'm sorry, {validation_message}")

    def voice_command_callback(self, msg):
        """Handle voice command from other nodes"""
        text = msg.data.lower().strip()
        self.process_recognized_text(text)

    def execute_command(self, parsed_command):
        """Execute the parsed command on the robot"""
        action = parsed_command['action']
        params = parsed_command['parameters']

        self.get_logger().info(f'Executing command: {action} with params: {params}')

        if action == 'move_forward':
            self.move_robot(0.3, 0.0)  # Move forward at 0.3 m/s
        elif action == 'move_backward':
            self.move_robot(-0.3, 0.0)  # Move backward at 0.3 m/s
        elif action == 'turn_left':
            self.move_robot(0.0, 0.5)  # Turn left at 0.5 rad/s
        elif action == 'turn_right':
            self.move_robot(0.0, -0.5)  # Turn right at 0.5 rad/s
        elif action == 'stop':
            self.stop_robot()
        elif action == 'navigate_to':
            location = params.get('location', 'unknown')
            self.navigate_to_location(location)
        elif action == 'bring_object':
            obj = params.get('object', 'unknown')
            self.fetch_object(obj)
        elif action == 'find_person':
            self.search_for_person()
        elif action == 'wave':
            self.perform_wave_action()
        else:
            self.get_logger().warn(f'Unknown action: {action}')

    def move_robot(self, linear_vel, angular_vel):
        """Move the robot with specified velocities"""
        cmd = Twist()
        cmd.linear.x = linear_vel
        cmd.angular.z = angular_vel

        self.cmd_vel_pub.publish(cmd)
        self.robot_state = 'moving'

    def stop_robot(self):
        """Stop the robot"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0

        self.cmd_vel_pub.publish(cmd)
        self.robot_state = 'idle'

    def navigate_to_location(self, location):
        """Navigate to a specified location (simplified)"""
        # In a real implementation, this would use navigation stack
        self.get_logger().info(f'Navigating to {location}')

        # For this lab, we'll just move forward as a placeholder
        self.move_robot(0.2, 0.0)

        # After some time, stop
        timer = self.create_timer(3.0, self.stop_robot)
        self.robot_state = 'navigating'

    def fetch_object(self, obj_name):
        """Fetch a specified object (simplified)"""
        self.get_logger().info(f'Fetching {obj_name}')

        # In a real implementation, this would involve:
        # 1. Localizing the object
        # 2. Planning a path to the object
        # 3. Approaching and grasping the object
        # 4. Returning to user

        # For this lab, we'll just acknowledge the request
        self.speech_synthesizer.speak(f"I'm looking for the {obj_name}. Please wait.")

        # Simulate the action
        timer = self.create_timer(2.0, lambda: self.speech_synthesizer.speak(f"I found the {obj_name} and will bring it to you."))

    def search_for_person(self):
        """Search for a person (simplified)"""
        self.get_logger().info('Searching for person')

        # In a real implementation, this would use computer vision
        # to detect and locate people

        self.speech_synthesizer.speak("I'm looking for a person. Please wait.")

        # Simulate finding a person
        timer = self.create_timer(2.0, lambda: self.speech_synthesizer.speak("I found a person. Hello!"))

    def perform_wave_action(self):
        """Perform a waving action (simplified)"""
        self.get_logger().info('Performing wave action')

        # In a real implementation, this would control arm joints
        # to perform a waving motion

        self.speech_synthesizer.speak("Hello! Nice to meet you!")

    def get_available_commands(self):
        """Get list of available voice commands"""
        commands = self.nlu.get_available_commands()
        return commands

    def toggle_listening(self):
        """Toggle voice listening on/off"""
        if self.is_listening:
            self.stop_voice_listening()
        else:
            self.start_voice_listening()

def main(args=None):
    rclpy.init(args=args)
    voice_interface = VoiceInterfaceNode()

    try:
        rclpy.spin(voice_interface)
    except KeyboardInterrupt:
        voice_interface.stop_voice_listening()
    finally:
        voice_interface.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Part 7: Creating a Dialogue Manager (35 minutes)

Create a dialogue manager for more sophisticated conversations:

```python
#!/usr/bin/env python3
import json
import time
from enum import Enum
from typing import Dict, List, Optional, Callable

class DialogueState(Enum):
    IDLE = "idle"
    LISTENING = "listening"
    PROCESSING = "processing"
    RESPONDING = "responding"
    WAITING_FOR_RESPONSE = "waiting_for_response"

class DialogueManager:
    def __init__(self, speech_synthesizer, speech_recognizer):
        self.speech_synthesizer = speech_synthesizer
        self.speech_recognizer = speech_recognizer

        self.state = DialogueState.IDLE
        self.context = {}
        self.conversation_history = []
        self.active_intent = None
        self.waiting_for_input = False
        self.response_callback = None

        # Define conversation flows
        self.conversation_flows = {
            'greeting': self.handle_greeting,
            'help': self.handle_help,
            'navigation': self.handle_navigation,
            'object_fetch': self.handle_object_fetch,
            'follow_up': self.handle_follow_up
        }

        # Define follow-up questions
        self.follow_up_questions = {
            'navigation': [
                "Would you like me to go there now?",
                "How urgent is this?",
                "Do you need anything else?"
            ],
            'object_fetch': [
                "Is this the right object?",
                "Would you like me to bring anything else?",
                "Where should I place it?"
            ]
        }

    def start_conversation(self):
        """Start a new conversation"""
        self.state = DialogueState.IDLE
        self.context = {}
        self.conversation_history = []

        # Greet the user
        greeting = self.get_greeting()
        self.speech_synthesizer.speak(greeting)

        self.add_to_history('system', greeting)

    def get_greeting(self) -> str:
        """Get an appropriate greeting based on time of day"""
        hour = time.localtime().tm_hour

        if 5 <= hour < 12:
            return "Good morning! I'm your robot assistant. How can I help you today?"
        elif 12 <= hour < 17:
            return "Good afternoon! I'm ready to assist you. What would you like me to do?"
        elif 17 <= hour < 21:
            return "Good evening! I'm here to help. How can I assist you?"
        else:
            return "Hello! I'm your robot assistant. How can I help you?"

    def handle_user_input(self, text: str):
        """Handle user input and determine appropriate response"""
        self.add_to_history('user', text)

        self.state = DialogueState.PROCESSING

        # Determine intent from user input
        intent = self.determine_intent(text)

        # Process the intent
        response = self.process_intent(intent, text)

        self.state = DialogueState.RESPONDING
        self.speech_synthesizer.speak(response)

        self.add_to_history('system', response)

        # Check if we need follow-up
        if self.needs_follow_up(intent):
            self.ask_follow_up(intent)

        self.state = DialogueState.IDLE

    def determine_intent(self, text: str) -> str:
        """Determine the intent from user text"""
        text_lower = text.lower()

        # Simple keyword-based intent detection
        if any(word in text_lower for word in ['hello', 'hi', 'hey', 'greetings']):
            return 'greeting'
        elif any(word in text_lower for word in ['help', 'what can you do', 'assist', 'commands']):
            return 'help'
        elif any(word in text_lower for word in ['go to', 'navigate', 'move to', 'go']):
            return 'navigation'
        elif any(word in text_lower for word in ['bring', 'get', 'fetch', 'pick up']):
            return 'object_fetch'
        elif any(word in text_lower for word in ['yes', 'yeah', 'sure', 'ok', 'okay']):
            return 'affirmative'
        elif any(word in text_lower for word in ['no', 'nope', 'not', 'don\'t']):
            return 'negative'
        else:
            return 'unknown'

    def process_intent(self, intent: str, text: str) -> str:
        """Process the determined intent and return response"""
        if intent in self.conversation_flows:
            return self.conversation_flows[intent](text)
        else:
            return self.handle_unknown_intent(text)

    def handle_greeting(self, text: str) -> str:
        """Handle greeting intent"""
        responses = [
            "Hello! It's nice to meet you.",
            "Hi there! How can I assist you?",
            "Greetings! I'm ready to help."
        ]
        import random
        return random.choice(responses)

    def handle_help(self, text: str) -> str:
        """Handle help intent"""
        return ("I can help you with several things: "
                "I can move around (forward, backward, turn), "
                "navigate to specific locations like kitchen or bedroom, "
                "fetch objects like water or books, "
                "and I can also wave or perform simple actions. "
                "What would you like me to do?")

    def handle_navigation(self, text: str) -> str:
        """Handle navigation intent"""
        # Extract location from text
        import re
        location_match = re.search(r'to (\w+)|navigate (\w+)|go (\w+)', text.lower())

        if location_match:
            location = location_match.group(1) or location_match.group(2) or location_match.group(3)
            self.context['target_location'] = location
            return f"I can help you go to the {location}."
        else:
            return "Where would you like me to go?"

    def handle_object_fetch(self, text: str) -> str:
        """Handle object fetch intent"""
        # Extract object from text
        import re
        object_match = re.search(r'bring (\w+)|get (\w+)|fetch (\w+)', text.lower())

        if object_match:
            obj = object_match.group(1) or object_match.group(2) or object_match.group(3)
            self.context['target_object'] = obj
            return f"I can help you get the {obj}."
        else:
            return "What object would you like me to fetch?"

    def handle_unknown_intent(self, text: str) -> str:
        """Handle unknown intent"""
        return "I'm not sure I understand. Could you please rephrase that?"

    def needs_follow_up(self, intent: str) -> bool:
        """Check if the intent needs follow-up questions"""
        return intent in self.follow_up_questions

    def ask_follow_up(self, intent: str):
        """Ask follow-up questions for the intent"""
        if intent in self.follow_up_questions:
            follow_ups = self.follow_up_questions[intent]
            if follow_ups:
                follow_up = follow_ups[0]  # Ask the first follow-up
                self.speech_synthesizer.speak(follow_up)
                self.add_to_history('system', follow_up)

                # Set up to listen for response
                self.waiting_for_input = True
                self.active_intent = intent

    def handle_follow_up(self, text: str) -> str:
        """Handle follow-up responses"""
        # Process the follow-up response
        if self.active_intent == 'navigation':
            # Handle navigation follow-up
            if 'yes' in text.lower():
                return f"Okay, I'll go to the {self.context.get('target_location', 'location')} now."
            elif 'no' in text.lower():
                return "Okay, let me know when you're ready."
            else:
                return "I'll proceed with the navigation."

        elif self.active_intent == 'object_fetch':
            # Handle object fetch follow-up
            if 'yes' in text.lower():
                return f"Great! I'll get the {self.context.get('target_object', 'object')} for you."
            else:
                return "Okay, is there anything else I can help with?"

        return "Thank you for your response."

    def add_to_history(self, speaker: str, text: str):
        """Add an utterance to the conversation history"""
        timestamp = time.time()
        self.conversation_history.append({
            'timestamp': timestamp,
            'speaker': speaker,
            'text': text
        })

    def get_conversation_context(self) -> Dict:
        """Get the current conversation context"""
        return {
            'state': self.state.value,
            'context': self.context,
            'history': self.conversation_history[-5:],  # Last 5 exchanges
            'active_intent': self.active_intent
        }

    def reset_conversation(self):
        """Reset the conversation to initial state"""
        self.state = DialogueState.IDLE
        self.context = {}
        self.active_intent = None
        self.waiting_for_input = False

# Example usage
if __name__ == "__main__":
    # This would be integrated with the main voice interface node
    print("Dialogue manager ready for integration")
```

### Part 8: Creating a Voice Command Tester (20 minutes)

Create a node to test voice commands:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from lab_voice_interface.speech_synthesizer import SpeechSynthesizer

class VoiceCommandTester(Node):
    def __init__(self):
        super().__init__('voice_command_tester')

        self.voice_pub = self.create_publisher(String, 'voice_command', 10)
        self.status_pub = self.create_publisher(String, 'test_status', 10)

        # Initialize speech synthesizer for feedback
        self.speech_synthesizer = SpeechSynthesizer()

        # Test commands
        self.test_commands = [
            "move forward",
            "turn left",
            "stop",
            "bring me water",
            "go to kitchen",
            "find person",
            "wave hello"
        ]

        self.current_test_index = 0
        self.testing_active = False

        # Timer for automated testing
        self.test_timer = None

        self.get_logger().info('Voice command tester initialized')

    def start_automated_test(self):
        """Start automated testing of voice commands"""
        self.testing_active = True
        self.current_test_index = 0
        self.test_timer = self.create_timer(3.0, self.run_next_test)
        self.get_logger().info('Starting automated voice command tests')

    def run_next_test(self):
        """Run the next test command"""
        if self.current_test_index >= len(self.test_commands):
            self.stop_testing()
            return

        command = self.test_commands[self.current_test_index]

        # Publish the command
        cmd_msg = String()
        cmd_msg.data = command
        self.voice_pub.publish(cmd_msg)

        # Log the test
        status_msg = String()
        status_msg.data = f"Testing command: {command}"
        self.status_pub.publish(status_msg)

        self.get_logger().info(f'Testing command: {command}')

        # Provide audio feedback
        self.speech_synthesizer.speak(f"Testing command: {command}")

        self.current_test_index += 1

    def stop_testing(self):
        """Stop the automated testing"""
        self.testing_active = False
        if self.test_timer:
            self.test_timer.cancel()

        status_msg = String()
        status_msg.data = "Testing completed"
        self.status_pub.publish(status_msg)

        self.speech_synthesizer.speak("Voice command testing completed")

        self.get_logger().info('Voice command testing completed')

    def manual_test_command(self, command: str):
        """Test a specific command manually"""
        cmd_msg = String()
        cmd_msg.data = command
        self.voice_pub.publish(cmd_msg)

        self.get_logger().info(f'Manual test: {command}')

def main(args=None):
    rclpy.init(args=args)
    tester = VoiceCommandTester()

    try:
        # Start automated testing
        tester.start_automated_test()

        rclpy.spin(tester)
    except KeyboardInterrupt:
        tester.stop_testing()
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Part 9: Creating Launch File (10 minutes)

Create a launch file to run the complete voice interface system:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lab_voice_interface',
            executable='voice_interface_node',
            name='voice_interface_node',
            parameters=[
                {'confirm_commands': True},
                {'response_timeout': 30.0}
            ],
            output='screen'
        ),
        Node(
            package='lab_voice_interface',
            executable='voice_command_tester',
            name='voice_command_tester',
            output='screen'
        )
    ])
```

### Part 10: Updating setup.py and Building (15 minutes)

Update the setup.py file:

```python
from setuptools import setup

package_name = 'lab_voice_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/voice_interface_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Voice interface for human-robot interaction lab',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'voice_interface_node = lab_voice_interface.voice_interface_node:main',
            'voice_command_tester = lab_voice_interface.voice_command_tester:main',
        ],
    },
)
```

Build the package:

```bash
cd ~/ros2_labs
colcon build --packages-select lab_voice_interface
source install/setup.bash
```

## Discussion Questions

1. How does speech recognition accuracy affect the overall user experience in human-robot interaction?

2. What are the challenges of implementing natural language understanding for robotics applications?

3. How can voice interfaces be made more accessible for users with different abilities?

4. What privacy considerations are important in voice-enabled robotic systems?

5. How would you handle ambiguous or unclear voice commands in a robotic system?

## Extension Activities

1. **Multilingual Support**: Extend the system to support multiple languages
2. **Emotion Recognition**: Add emotion detection from voice patterns
3. **Gesture Integration**: Combine voice with gesture-based commands
4. **Context Awareness**: Implement context-aware responses based on environment
5. **Learning System**: Create a system that learns user preferences over time

## Assessment

Complete the following self-assessment:

- I can implement speech recognition for robot commands: [ ] Yes [ ] No [ ] Partially
- I understand natural language processing for HRI: [ ] Yes [ ] No [ ] Partially
- I can create speech synthesis for robot responses: [ ] Yes [ ] No [ ] Partially
- I can integrate voice interface with ROS2: [ ] Yes [ ] No [ ] Partially
- I understand dialogue management principles: [ ] Yes [ ] No [ ] Partially

## Troubleshooting Tips

- **Audio Issues**: Check microphone permissions and audio drivers
- **Recognition Accuracy**: Improve with noise cancellation and proper microphone placement
- **Timing Issues**: Ensure proper synchronization between speech recognition and response
- **Background Noise**: Use noise reduction techniques and proper acoustic environment
- **Latency**: Optimize processing pipeline for real-time response

## Conclusion

This lab has provided hands-on experience with voice interfaces for human-robot interaction, including speech recognition, natural language understanding, and speech synthesis. You've learned to create an integrated voice command system that enables natural communication with robots.

## Resources for Further Exploration

- Speech recognition libraries: SpeechRecognition, Vosk, Google Cloud Speech-to-Text
- Text-to-speech libraries: pyttsx3, Google Cloud Text-to-Speech, Amazon Polly
- Natural language processing: spaCy, NLTK, transformers
- ROS2 sound packages for audio processing
- Research papers on voice interfaces for robotics
- Accessibility guidelines for voice user interfaces