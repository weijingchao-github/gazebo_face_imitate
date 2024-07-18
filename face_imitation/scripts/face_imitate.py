import math
import os

import cv2
import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelConfiguration, SetModelConfigurationRequest
from geometry_msgs.msg import Point, Quaternion, Vector3
from pynput import keyboard
from scipy.spatial.transform import Rotation


class HeadStateSetterError(Exception):
    pass


def on_press(key):
    if key.char == "q":
        return False
    else:
        print("\nPress Q to start face imitation.")


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
    image_name_sequence = []
    blink_sequence = []
    head_pose_yaw_angle_sequence = []
    head_pose_pitch_angle_sequence = []
    eye_left_horizontal_position_ratio_sequence = []
    eye_left_vertical_position_ratio_sequence = []
    eye_right_horizontal_position_ratio_sequence = []
    eye_right_vertical_position_ratio_sequence = []

    data_pieces_cnt = 0
    with open(
        os.path.join(os.path.dirname(__file__), "smooth_yzx.txt"), "r", encoding="utf-8"
    ) as source_file:
        for data_piece_num, data_one_piece in enumerate(source_file):
            if data_piece_num == 0:
                continue
            data_one_piece = data_one_piece.split()
            data_pieces_cnt += 1
            image_name_sequence.append(data_one_piece[0])
            blink_sequence.append(data_one_piece[1])
            head_pose_yaw_angle_sequence.append(float(data_one_piece[2]))
            head_pose_pitch_angle_sequence.append(float(data_one_piece[3]))
            eye_left_horizontal_position_ratio_sequence.append(float(data_one_piece[7]))
            eye_left_vertical_position_ratio_sequence.append(float(data_one_piece[8]))
            eye_right_horizontal_position_ratio_sequence.append(
                float(data_one_piece[5])
            )
            eye_right_vertical_position_ratio_sequence.append(float(data_one_piece[6]))

    rospy.loginfo("Read data from txt finished.")

    # conduct face imitation
    total_time_steps = data_pieces_cnt
    eye_horizontal_length = 0.028  # unit: m
    eye_vertical_length = 0.0113  # unit: m
    eye_radius = 0.01464  # unit: m

    blink_flag = False
    blink_procedures = [-60, -45, 0, -45, -60, -90]  # unit: degrees
    blink_progress = 0
    blink_continuous_cnt = 0

    # show image
    image_dir_path = os.path.join(os.path.dirname(__file__), "wjc_video3")
    cv2.namedWindow("video", cv2.WINDOW_GUI_NORMAL)
    image_resolution = (1920, 1080)
    cv2.resizeWindow(
        "video", int(image_resolution[0] * 0.4), int(image_resolution[1] * 0.4)
    )
    cv2.moveWindow("video", 0, 0)

    while True:
        rospy.loginfo("Press Q to start face imitation")
        with keyboard.Listener(on_press=on_press) as listener:
            listener.join()

        for time_step in range(total_time_steps):
            # head pitch and yaw joint state synthesis
            head_pose_pitch_angle = math.radians(
                head_pose_pitch_angle_sequence[time_step]
            )
            head_pose_yaw_angle = math.radians(head_pose_yaw_angle_sequence[time_step])
            head_state_srv_request = SetModelConfigurationRequest()
            head_state_srv_request.model_name = "head_yaw_and_pitch"
            head_state_srv_request.joint_names = ["joint_pitch", "joint_yaw"]
            head_state_srv_request.joint_positions = [
                head_pose_pitch_angle,
                head_pose_yaw_angle,
            ]  # radian

            # eye state msg synthesis
            eye_left_state_msg = ModelState()
            eye_left_state_msg.model_name = "eye_left"
            eye_left_position = [0.07506, -0.03055, 0.12254]  # xyz
            eye_left_state_msg.pose.position = Point(*eye_left_position)
            eye_left_position_x = eye_radius
            eye_left_position_y = (
                eye_left_horizontal_position_ratio_sequence[time_step]
                * eye_horizontal_length
            )
            eye_left_position_z = (
                eye_left_vertical_position_ratio_sequence[time_step]
                * eye_vertical_length
            )
            eye_left_euler_angle_z = math.atan(
                eye_left_position_y / eye_left_position_x
            )
            eye_left_euler_angle_y = math.atan(
                -eye_left_position_z
                / math.sqrt(eye_left_position_x**2 + eye_left_position_y**2)
            )
            eye_left_euler_angle_x = 0
            eye_left_euler_angle_zyx = [
                eye_left_euler_angle_z,
                eye_left_euler_angle_y,
                eye_left_euler_angle_x,
            ]
            quaternion = Rotation.from_euler(
                "zyx", eye_left_euler_angle_zyx, degrees=False
            ).as_quat()  # xyzw
            eye_left_state_msg.pose.orientation = Quaternion(*quaternion)
            eye_left_state_msg.reference_frame = "link_head"

            eye_right_state_msg = ModelState()
            eye_right_state_msg.model_name = "eye_right"
            eye_right_position = [0.07506, 0.03055, 0.12254]  # xyz
            eye_right_state_msg.pose.position = Point(*eye_right_position)
            eye_right_position_x = eye_radius
            eye_right_position_y = (
                eye_right_horizontal_position_ratio_sequence[time_step]
                * eye_horizontal_length
            )
            eye_right_position_z = (
                eye_right_vertical_position_ratio_sequence[time_step]
                * eye_vertical_length
            )
            eye_right_euler_angle_z = math.atan(
                eye_right_position_y / eye_right_position_x
            )
            eye_right_euler_angle_y = math.atan(
                -eye_right_position_z
                / math.sqrt(eye_right_position_x**2 + eye_right_position_y**2)
            )
            eye_right_euler_angle_x = 0
            eye_right_euler_angle_zyx = [
                eye_right_euler_angle_z,
                eye_right_euler_angle_y,
                eye_right_euler_angle_x,
            ]
            quaternion = Rotation.from_euler(
                "zyx", eye_right_euler_angle_zyx, degrees=False
            ).as_quat()
            eye_right_state_msg.pose.orientation = Quaternion(*quaternion)
            eye_right_state_msg.reference_frame = "link_head"

            # eyelid state msg synthesis
            if (
                blink_sequence[time_step] == "True"
            ):  # 针对眨眼很慢,有连续好几帧都在眨眼这种情况的判断
                blink_continuous_cnt += 1
                if blink_continuous_cnt > len(blink_procedures):
                    blink_flag = False
                else:
                    blink_flag = True
            else:
                blink_continuous_cnt = 0

            if blink_flag:
                blink_angle = blink_procedures[blink_progress]
                blink_progress += 1
                if (blink_progress + 1) > len(blink_procedures):
                    blink_flag = False
                    blink_progress = 0
            else:
                blink_angle = -90

            eyelid_left_state_msg = ModelState()
            eyelid_left_state_msg.model_name = "eyelid_left"
            eyelid_left_position = [0.07506, -0.03055, 0.12254]  # xyz
            eyelid_left_state_msg.pose.position = Point(*eyelid_left_position)
            eyelid_left_euler_angle_xyz = [0, blink_angle, 0]
            quaternion = Rotation.from_euler(
                "xyz", eyelid_left_euler_angle_xyz, degrees=True
            ).as_quat()  # xyzw
            eyelid_left_state_msg.pose.orientation = Quaternion(*quaternion)
            eyelid_left_state_msg.reference_frame = "link_head"

            eyelid_right_state_msg = ModelState()
            eyelid_right_state_msg.model_name = "eyelid_right"
            eyelid_right_position = [0.07506, 0.03055, 0.12254]  # xyz
            eyelid_right_state_msg.pose.position = Point(*eyelid_right_position)
            eyelid_right_euler_angle_xyz = [0, blink_angle, 0]
            quaternion = Rotation.from_euler(
                "xyz", eyelid_right_euler_angle_xyz, degrees=True
            ).as_quat()
            eyelid_right_state_msg.pose.orientation = Quaternion(*quaternion)
            eyelid_right_state_msg.reference_frame = "link_head"

            # call and pub
            head_state_setter_result = head_state_setter.call(head_state_srv_request)
            if not head_state_setter_result.success:
                raise HeadStateSetterError
            eye_and_eyelid_state_setter.publish(eye_left_state_msg)
            eye_and_eyelid_state_setter.publish(eye_right_state_msg)
            eye_and_eyelid_state_setter.publish(eyelid_left_state_msg)
            eye_and_eyelid_state_setter.publish(eyelid_right_state_msg)

            # show and update image
            image_name = image_name_sequence[time_step]
            image = cv2.imread(os.path.join(image_dir_path, f"{image_name}"))
            image = cv2.resize(
                image, None, fx=0.4, fy=0.4, interpolation=cv2.INTER_CUBIC
            )
            cv2.imshow("video", image)
            cv2.waitKey(1)

            # delay
            rospy.sleep(rospy.Duration(1 / 30))


if __name__ == "__main__":
    main()
