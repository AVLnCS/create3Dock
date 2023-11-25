import rclpy
from rclpy.node import Node
from rclpy.action.client import ActionClient
from rclpy.duration import Duration

import irobot_create_msgs
# Added RotateAngle
from irobot_create_msgs.action import DriveDistance, Undock, RotateAngle, AudioNoteSequence
from irobot_create_msgs.msg import AudioNote, AudioNoteVector


from pynput.keyboard import KeyCode
from key_commander import KeyCommander


class Roomba(Node):
    def __init__(self, namespace):
        super().__init__('robot')  # Init parent and give it a name
        self._undock_ac = ActionClient(self, Undock, f'/{namespace}/undock')
        self._drive_ac = ActionClient(self, DriveDistance, f'/{namespace}/drive_distance')

        # Added by Gary Below
        self._rotate_ac = ActionClient(self, RotateAngle, f'/{namespace}/rotate_angle')
        # Initialize the audio publisher
        self.publisher_ = self.create_publisher(AudioNoteVector, f'/{namespace}/audio_note_vector', 10)
        self._audio_note_sequence_ac = ActionClient(self, AudioNoteSequence, f'/{namespace}/audio_note_sequence')
    def drive_away(self):
        """
        Undocks the robot using the blocking version of send_goal()
        so that the robot doesn't attempt to drive until the
        goal is complete./home/gary/Documents/robots

        Afterward an asynchronous goal is sent which allows
        the goal to be cancelled before it finishes executing.
        """
        self.get_logger().warning('WAITING FOR SERVER')
        # wait until the robot server is found and
        #   ready to receive a new goal
        self._undock_ac.wait_for_server()
        self.get_logger().warning('SERVER AVAILABLE')
        self.get_logger().warning('UNDOCKING')

        # create new Undock goal object to send to server
        undock_goal = Undock.Goal()

        # send the goal: send_goal() blocks until action
        #   is complete without returning a UUID for the goal.
        #   Thus, it is impossible to cancel the goal from this
        #   script.
        self._undock_ac.send_goal(undock_goal)

        # print statement after goal completes since send_goal() blocks
        self.get_logger().warning('UNDOCKED')

        # wait for DriveDistance action server (blocking)
        self._drive_ac.wait_for_server()
        self.get_logger().warning('DRIVING!')

        # create goal object and specify distance to drive
        drive_goal = DriveDistance.Goal()
        drive_goal.distance = 0.5

        # send goal async
        drive_goal = DriveDistance.Goal()
        # Added by Gary
        self._drive_ac.send_goal(drive_goal)

    def play_note_sequence(self):
        # Wait for the action server to be available
        self._audio_note_sequence_ac.wait_for_server()

        # Define a sequence of notes with their frequencies in Hz and durations in seconds
        notes_info = [

            (440, 0.5),  # A4 for 0.5 seconds
            (440, 0.5),  # A4 for 0.5 seconds
            (440, 0.5),  # A4 for 0.5 seconds
            (349, 0.35),  # F4 for 0.4 seconds
            (523, 0.15),  # C5 for 0.1 seconds
            (440, 0.5),  # A4 for 0.5 seconds
            (349, 0.35),  # F4 for 0.4 seconds
            (523, 0.15),  # C5 for 0.1 seconds
            (440, 1.0),  # A4 for 1.0 seconds

            (659, 0.5),  # E5 for 0.5 seconds
            (659, 0.5),  # E5 for 0.5 seconds
            (659, 0.5),  # E5 for 0.5 seconds
            (698, 0.35),  # F5 for 0.4 seconds
            (523, 0.15),  # C5 for 0.1 seconds
            (415, 0.5),  # Ab4 for 0.5 seconds
            (349, 0.35),  # F4 for 0.4 seconds
            (523, 0.15),  # C5 for 0.1 seconds
            (440, 1.0),  # A4 for 1.0 seconds


            (880, 0.5),  # A5 for 1.0 seconds
            (440, 0.35),  # A4 for 1.0 seconds
            (440, 0.15),  # A4 for 1.0 seconds
            (880, 0.5),  # A5 for 1.0 seconds
            (830, 0.35),  # Ab5 for 0.5 seconds
            (775, 0.15),  # G5 for 0.5 seconds
            (740, 0.15),  # Gb5 for 0.5 seconds
            (698, 0.15),  # F5 for 0.5 seconds
            (740, 0.35),  # Gb5 for 0.5 seconds
            (466, 0.25),  # Bb4 for 0.5 seconds
            (622, 0.5),  # Eb for 0.5 seconds

        ]

        # Create AudioNoteVector
        note_vector = AudioNoteVector()
        for frequency, duration in notes_info:
            note = AudioNote()
            note.frequency = frequency
            note.max_runtime = Duration(seconds=duration).to_msg()
            note_vector.notes.append(note)

        # Create an AudioNoteSequence goal
        goal = AudioNoteSequence.Goal()
        goal.iterations = 1  # Number of times to play the sequence
        goal.note_sequence = note_vector

        # Send the goal
        # self._audio_note_sequence_ac.send_goal_async(goal)
        # self.get_logger().info('Playing note sequence...')

    def rotate(self, future):
        drive_result = future.result()

        if drive_result is None:
            self.get_logger().error("Drive action did not complete successfully.")
            return

        self.get_logger().info(f"Drive action completed with result: {drive_result}")
        self.get_logger().warning('STARTING ROTATION')

        rotate_goal = RotateAngle.Goal()
        rotate_goal.angle = 1.53
        self._rotate_ac.send_goal(rotate_goal)


if __name__ == '__main__':
    rclpy.init()
    namespace = 'create3_0620'
    s = Roomba(namespace)

    keycom = KeyCommander([
        (KeyCode(char='s'), s.drive_away),
        # (KeyCode(char='r'), s.rotate),
        # (KeyCode(char='m'), s.play_note_sequence())
    ])

    print("s: dock and drive a meter")
    print("m: Music")

    rclpy.spin(s)
    rclpy.shutdown()
