#!/home/jstm/Projects/mpcm2/deoxys_venv/bin/python3
import sys
print("Python interpreter:", sys.executable)
import argparse
import rospy
from sensor_msgs.msg import JointState
from deoxys.franka_interface import FrankaInterface
import numpy as np
from pathlib import Path


class PandaJointStatePublisher:
    def __init__(self, interface_cfg_filepath: Path):
        print(f"Initializing FrankaInterface with config file: {interface_cfg_filepath}")
        # Create FrankaInterface following deoxys example
        self.franka_interface = FrankaInterface(interface_cfg_filepath, use_visualizer=False)

        # Create joint state publishers
        self.joint_state_pub = rospy.Publisher('joint_states', JointState, queue_size=10)

        # Define joint names
        self.joint_names = [
            'panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4',
            'panda_joint5', 'panda_joint6', 'panda_joint7',
            'panda_finger_joint1', 'panda_finger_joint2'
        ]

        self.rate = rospy.Rate(10)  # 10 Hz

    def run(self):
        counter = 0
        while not rospy.is_shutdown():
            self.rate.sleep()
            try:
                # Publish robot joint states
                arm_q = self.franka_interface.last_q
                gripper_angle = float(self.franka_interface.last_gripper_q) / 2.0
                if arm_q is None or gripper_angle is None:
                    print("Arm or gripper angle is None")
                    continue
                js_msg = JointState()
                js_msg.header.stamp = rospy.Time.now()
                js_msg.name = self.joint_names
                assert isinstance(gripper_angle, float), f"Gripper angle is not a float: {type(gripper_angle)=}, {gripper_angle=}"
                js_msg.position = arm_q.tolist() + [gripper_angle, gripper_angle]
                js_msg.velocity = self.franka_interface.last_dq.tolist() + [0.0, 0.0]
                js_msg.effort = [0.0] * 9
                self.joint_state_pub.publish(js_msg)

                if counter % 50 == 0:
                    print("\n==================\n")
                    print(js_msg)
                    print("___")
                counter += 1

            except Exception as e:
                rospy.logerr(f"Error publishing joint states: [type: {type(e)}] {e}")
                break



def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("deoxys_interface_cfg_filepath", type=str)
    args, _ = parser.parse_known_args()
    rospy.init_node('panda_joint_state_publisher', anonymous=False)
    interface_cfg_filepath = Path(args.deoxys_interface_cfg_filepath.split(":=")[1])
    assert interface_cfg_filepath.exists(), f"Interface config file {interface_cfg_filepath} does not exist"
    publisher = PandaJointStatePublisher(interface_cfg_filepath)
    publisher.run()

if __name__ == '__main__':
    main()
    