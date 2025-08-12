#!/home/jstm/Projects/mpcm2/deoxys_venv/bin/python3
import argparse
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
import numpy as np
from pathlib import Path

from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
import tf.transformations as tft
from geometry_msgs.msg import PoseStamped, Pose, Quaternion, Point

def make_6dof_marker(base_frame: str, child_frame: str, init_pose: Pose):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = base_frame
    int_marker.name = child_frame
    int_marker.description = f"Movable {child_frame} Frame"
    int_marker.scale = 0.3

    # Add visible arrow marker
    marker = Marker()
    marker.type = Marker.ARROW
    marker.scale.x = 0.05
    marker.scale.y = 0.02
    marker.scale.z = 0.02
    marker.color.r = 1.0
    marker.color.a = 1.0

    control = InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append(marker)
    int_marker.controls.append(control)
    
    # MOVE X (default orientation)
    control_x = InteractiveMarkerControl()
    control_x.name = "move_x"
    control_x.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    control_x.orientation.w, control_x.orientation.x, control_x.orientation.y, control_x.orientation.z = 1, 0, 0, 0

    # MOVE Y (rotate 90 deg around Z)
    control_y = InteractiveMarkerControl()
    control_y.name = "move_y"
    control_y.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    qx, qy, qz, qw = tft.quaternion_from_euler(0, 0, 1.5708)  # roll=0, pitch=0, yaw=90deg
    control_y.orientation.w = qw
    control_y.orientation.x = qx
    control_y.orientation.y = qy
    control_y.orientation.z = qz

    # MOVE Z (rotate -90 deg around Y)
    control_z = InteractiveMarkerControl()
    control_z.name = "move_z"
    control_z.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    control_z.orientation.w, control_z.orientation.x, control_z.orientation.y, control_z.orientation.z = tft.quaternion_from_euler(0, -1.57, 0)

    # ROTATE X (default orientation)
    rot_x = InteractiveMarkerControl()
    rot_x.name = "rotate_x"
    rot_x.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    rot_x.orientation.w, rot_x.orientation.x, rot_x.orientation.y, rot_x.orientation.z = 1, 0, 0, 0

    # ROTATE Y (rotate 90 deg around Z)
    rot_y = InteractiveMarkerControl()
    rot_y.name = "rotate_y"
    rot_y.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    qx, qy, qz, qw = tft.quaternion_from_euler(0, 0, 1.5708)  # roll=0, pitch=0, yaw=90deg
    rot_y.orientation.w = qw
    rot_y.orientation.x = qx
    rot_y.orientation.y = qy
    rot_y.orientation.z = qz

    # # ROTATE Z (rotate -90 deg around Y)
    rot_z = InteractiveMarkerControl()
    rot_z.name = "rotate_z"
    rot_z.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    rot_z.orientation.w, rot_z.orientation.x, rot_z.orientation.y, rot_z.orientation.z = tft.quaternion_from_euler(0, -1.57, 0)

    # Set initial pose
    int_marker.controls += [control_x, control_y, control_z, rot_x, rot_y, rot_z]
    int_marker.pose = init_pose
    return int_marker


class TFPublisher:
    def __init__(self):
        self.camera_frame_id = "camera_depth_optical_frame"
        self.robot_base_frame_id = "panda_link0"

        # Set up TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.tf_rate = rospy.Rate(50)

        # Set up interactive marker server
        self.last_pose = Pose(orientation=Quaternion(x=-0.7441695332527161,  y=-0.5852000713348389,  z=0.10598380863666534,  w=0.30417126417160034), position=Point(x=0.9778818488121033, y=-0.3103826344013214,  z=0.6897751092910767))
        self.server = InteractiveMarkerServer("movable_realsense_frame_marker")
        marker = make_6dof_marker(self.robot_base_frame_id, self.camera_frame_id, self.last_pose)
        self.server.insert(marker, self.process_feedback)
        self.server.applyChanges()


    def process_feedback(self, feedback):
        pose_msg = PoseStamped()
        pose_msg.header = feedback.header
        pose_msg.pose = feedback.pose
        self.last_pose = pose_msg.pose
        print("process_feedback:", self.last_pose)


    def run(self):
        rospy.loginfo(f"Publishing transform from {self.robot_base_frame_id} to {self.camera_frame_id}")

        while not rospy.is_shutdown():
            # Update timestamp
            tf = TransformStamped()
            tf.header.stamp = rospy.Time.now()
            # Fixed transform from robot base to camera (you can modify these values)
            # This is an example transform - replace with your actual calibration values
            tf.header.frame_id = self.robot_base_frame_id
            # tf.header.frame_id = "base_link"
            tf.child_frame_id = self.camera_frame_id
            tf.transform.translation.x = self.last_pose.position.x
            tf.transform.translation.y = self.last_pose.position.y
            tf.transform.translation.z = self.last_pose.position.z
            tf.transform.rotation.x = self.last_pose.orientation.x
            tf.transform.rotation.y = self.last_pose.orientation.y
            tf.transform.rotation.z = self.last_pose.orientation.z
            tf.transform.rotation.w = self.last_pose.orientation.w

            # Publish the transform
            self.tf_broadcaster.sendTransform(tf)

            # Publish base__T__panda_link0
            tf_base = TransformStamped()
            tf_base.header.stamp = rospy.Time.now()
            tf_base.header.frame_id = self.robot_base_frame_id
            tf_base.child_frame_id = "base_link"
            tf_base.transform.translation.x = 0.0
            tf_base.transform.translation.y = 0.0
            tf_base.transform.translation.z = 0.0
            tf_base.transform.rotation.x = 0.0
            tf_base.transform.rotation.y = 0.0
            tf_base.transform.rotation.z = 0.0
            tf_base.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(tf_base)
            self.tf_rate.sleep()


def main():
    parser = argparse.ArgumentParser(description="Publish fixed transform between camera and robot base")
    args, _ = parser.parse_known_args()
    
    rospy.init_node('camera_robot_tf_publisher', anonymous=False)
    
    publisher = TFPublisher()
    publisher.run()

if __name__ == '__main__':
    main()
    