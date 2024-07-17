import os
import sys

path = os.path.dirname(__file__)
sys.path.insert(0, path)

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelConfiguration, SetModelConfigurationRequest
from geometry_msgs.msg import Point, Quaternion, Vector3
from pynput import keyboard


def main():
    # ROS init
    rospy.init_node("face_imitate")
    head_state_setter = rospy.ServiceProxy(
        "/gazebo/set_model_configuration", SetModelConfiguration
    )
    head_state_setter.wait_for_service()
    eye_and_eyelid_state_setter = rospy.Publisher(
        "/gazebo/set_model_state", ModelState, queue_size=4
    )

    # wait for gazebo server to start and set initial state finished
    rospy.sleep(rospy.Duration(5))

    # read time sequence from .txt file
    with open("euler_angle_yzx_adjust.txt", "r", encoding="utf-8") as source_file:
        for i, data_one_piece in enumerate(source_file):
            if i <= 4
        
    rospy.loginfo("Read data from txt finshed.")

    # press Q to start face imitation
    def on_press(key):
        if key.char == "q":
            return False
        else:
            print("\nPress Q to start face imitation.")

    with keyboard.Listener(on_press=on_press) as listener:
        listener.join()


if __name__ == "__main__":
    main()
