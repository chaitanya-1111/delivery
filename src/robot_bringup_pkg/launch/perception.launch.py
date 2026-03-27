from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Microphone
        Node(
            package='audio_input_pkg',
            executable='microphone_node',
            name='microphone',
            output='screen'
        ),
        
        # 2. Camera (ESP32)
        Node(
            package='esp32_camera_pkg',
            executable='wifi_cam_node',
            name='esp32_camera',
            output='screen'
        ),
        
        # 3. Face Detector
        Node(
            package='face_detection_pkg',
            executable='face_detector_node',
            name='face_detector',
            output='screen'
        ),

        # 4. Face Tracker
        Node(
            package='face_tracking_pkg',
            executable='face_tracker_node',
            name='face_tracker',
            output='screen'
        ),

        # 5. Servo Controller
        Node(
            package='servo_control_pkg',
            executable='servo_node',
            name='servo_controller',
            output='screen'
        ),

        # 6. Session Manager (THE NEW BRAIN)
        Node(
            package='session_manager_pkg',
            executable='session_node',
            name='session_manager',
            output='screen'
        ),

        # 7. TTS Player
        Node(
            package='tts_player_pkg',
            executable='tts_node',
            name='tts_player',
            output='screen'
        ),

        # ... inside generate_launch_description ...
        # 8. Speech to Text (The Ear)
        Node(
            package='speech_to_text_pkg',
            executable='stt_node',
            name='stt_node',
            output='screen',
            # prefix='xterm -e' # OPTIONAL: Opens a separate terminal for typing input
        ),
        # ...
        # 9. AI Dialog (The Brain's Creative Center)
        Node(
            package='ai_dialog_pkg',
            executable='ai_node',
            name='ai_dialog',
            output='screen'
        ),
        # ... existing nodes ...
        
        # 10. Intent Classifier (The Translator)
        Node(
            package='intent_classifier_pkg',
            executable='classifier_node',
            name='intent_classifier',
            output='screen'
        )
    ])