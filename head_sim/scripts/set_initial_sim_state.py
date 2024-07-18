import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelConfiguration, SetModelConfigurationRequest
from geometry_msgs.msg import Point, Quaternion, Vector3
from scipy.spatial.transform import Rotation


class HeadStateSetterError(Exception):
    pass


def set_head_initial_state_srv_request():
    head_initial_state_srv_request = SetModelConfigurationRequest()
    head_initial_state_srv_request.model_name = "head_yaw_and_pitch"
    head_initial_state_srv_request.joint_names = ["joint_pitch", "joint_yaw"]
    head_initial_state_srv_request.joint_positions = [0, 0]  # radian
    return head_initial_state_srv_request


def set_eye_initial_state_msg():
    eye_left_initial_state_msg = ModelState()
    eye_left_initial_state_msg.model_name = "eye_left"
    eye_left_position = [0.07506, -0.03055, 0.12254]  # xyz
    eye_left_initial_state_msg.pose.position = Point(*eye_left_position)
    eye_left_euler_angle_zyx = [0, 0, 0]
    quaternion = Rotation.from_euler(
        "zyx", eye_left_euler_angle_zyx, degrees=True
    ).as_quat()  # xyzw
    eye_left_initial_state_msg.pose.orientation = Quaternion(*quaternion)
    eye_left_initial_state_msg.reference_frame = "link_head"

    eye_right_initial_state_msg = ModelState()
    eye_right_initial_state_msg.model_name = "eye_right"
    eye_right_position = [0.07506, 0.03055, 0.12254]  # xyz
    eye_right_initial_state_msg.pose.position = Point(*eye_right_position)
    eye_right_euler_angle_zyx = [0, 0, 0]
    quaternion = Rotation.from_euler(
        "zyx", eye_right_euler_angle_zyx, degrees=True
    ).as_quat()
    eye_right_initial_state_msg.pose.orientation = Quaternion(*quaternion)
    eye_right_initial_state_msg.reference_frame = "link_head"

    return eye_left_initial_state_msg, eye_right_initial_state_msg


def set_eyelid_initial_state_msg():
    eyelid_left_initial_state_msg = ModelState()
    eyelid_left_initial_state_msg.model_name = "eyelid_left"
    eyelid_left_position = [0.07506, -0.03055, 0.12254]  # xyz
    eyelid_left_initial_state_msg.pose.position = Point(*eyelid_left_position)
    eyelid_left_euler_angle_xyz = [0, -90, 0]
    quaternion = Rotation.from_euler(
        "xyz", eyelid_left_euler_angle_xyz, degrees=True
    ).as_quat()  # xyzw
    eyelid_left_initial_state_msg.pose.orientation = Quaternion(*quaternion)
    eyelid_left_initial_state_msg.reference_frame = "link_head"

    eyelid_right_initial_state_msg = ModelState()
    eyelid_right_initial_state_msg.model_name = "eyelid_right"
    eyelid_right_position = [0.07506, 0.03055, 0.12254]  # xyz
    eyelid_right_initial_state_msg.pose.position = Point(*eyelid_right_position)
    eyelid_right_euler_angle_xyz = [0, -90, 0]
    quaternion = Rotation.from_euler(
        "xyz", eyelid_right_euler_angle_xyz, degrees=True
    ).as_quat()
    eyelid_right_initial_state_msg.pose.orientation = Quaternion(*quaternion)
    eyelid_right_initial_state_msg.reference_frame = "link_head"

    return eyelid_left_initial_state_msg, eyelid_right_initial_state_msg


def main():
    # ROS init
    rospy.init_node("set_initial_sim_state")
    head_initial_state_setter = rospy.ServiceProxy(
        "/gazebo/set_model_configuration", SetModelConfiguration
    )
    head_initial_state_setter.wait_for_service()
    eye_and_eyelid_initial_state_setter = rospy.Publisher(
        "/gazebo/set_model_state", ModelState, queue_size=4
    )

    # wait for gazebo server to launch
    rospy.sleep(rospy.Duration(3))

    # set head, eye, eyelid initial state srv_request/msg
    head_initial_state_srv_request = set_head_initial_state_srv_request()
    eye_left_initial_state_msg, eye_right_initial_state_msg = (
        set_eye_initial_state_msg()
    )
    eyelid_left_initial_state_msg, eyelid_right_initial_state_msg = (
        set_eyelid_initial_state_msg()
    )

    # call and pub
    head_state_setter_result = head_initial_state_setter.call(
        head_initial_state_srv_request
    )
    if not head_state_setter_result.success:
        raise HeadStateSetterError
    eye_and_eyelid_initial_state_setter.publish(eye_left_initial_state_msg)
    eye_and_eyelid_initial_state_setter.publish(eye_right_initial_state_msg)
    eye_and_eyelid_initial_state_setter.publish(eyelid_left_initial_state_msg)
    eye_and_eyelid_initial_state_setter.publish(eyelid_right_initial_state_msg)


if __name__ == "__main__":
    main()
