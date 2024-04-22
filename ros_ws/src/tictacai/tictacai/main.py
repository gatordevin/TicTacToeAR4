import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import String
import os

# from gym_tictactoe.examples.td_agent import load_model
# from gym_tictactoe.gym_tictactoe.env import TicTacToeEnv

from .td_agent import load_model, TDAgent
from gym_tictactoe.env import TicTacToeEnv

MODEL_PATH = os.path.join(os.path.dirname(__file__), 'best_td_agent.dat')

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__("ai_node")
        print("Loaded model file: {}".format(MODEL_PATH))
        self.model = load_model(MODEL_PATH)

        self.publisher_ = self.create_publisher(Int32, "action", 10)
        self.subscription = self.create_subscription(
            String, "board", self.play, 10
        )

    def parsemsg(self, msgstr):
        outarr = [msgstr[0], msgstr[1], msgstr[2],
                  msgstr[3], msgstr[4], msgstr[5],
                  msgstr[6], msgstr[7], msgstr[8]]
        outturn = msgstr[9]
        return (outarr, outturn)

    def play(self, msg):
        # Create environment from board and turn
        env = TicTacToeEnv()
        env.reset()
        env.start_mark = "X"

        # Read board and turn from message
        board_char, turn_char = self.parsemsg(msg.data)
        print("Received message: {}".format(msg.data))
        board_int = list([1 if i=="O" else 2 if i=="X" else 0 for i in board_char])
        env.board = board_int

        if turn_char != "X":
            print("Invalid turn character. Ignoring message...")
            # Return if not robot turn
            return

        td_agent = TDAgent('X', 0, 0)  # prevent exploring

        ava = env.available_actions()

        # Choose action
        action = td_agent.act(
            (env.board, env.start_mark), ava
        )

        # Publish that action
        action_msg = Int32()
        action_msg.data = action
        self.publisher_.publish(action_msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
