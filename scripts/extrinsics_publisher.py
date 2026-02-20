#!/home/jstm/Projects/mpcm2/deoxys_venv/bin/python3
import argparse
from re import M
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
import numpy as np
from pathlib import Path

from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
import tf.transformations as tft
from geometry_msgs.msg import PoseStamped, Pose, Quaternion, Point

np.set_printoptions(precision=4, suppress=True)

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

def _transformation_matrix_to_pose(transformation_matrix: np.ndarray) -> Pose:
    # From quaternion_from_matrix returns a quaternion as[x, y, z, w].
    q = tft.quaternion_from_matrix(transformation_matrix)
    return Pose(
        orientation=Quaternion(
            x=q[0],
            y=q[1],
            z=q[2],
            w=q[3]
        ),
        position=Point(
            x=transformation_matrix[0, 3],
            y=transformation_matrix[1, 3],
            z=transformation_matrix[2, 3]
        )
    )


# panda_link0__T__${camera_name}_base_link
EXTRINSICS_MATRICES = {
    "camera_south": np.array([
        [-0.6379, -0.7203,  0.2726,  1.0278],
        [ 0.2826,  0.1104,  0.9529, -0.3099],
        [-0.7164,  0.6849,  0.1331,  0.7441],
        [ 0.0,     0.0,     0.0,     1.0]
    ]),
    "camera_north": np.array([
        [-0.4504,  0.8766,  0.1694,  0.887],
        [-0.3919, -0.0236, -0.9197,  0.4744],
        [-0.8022, -0.4806,  0.3542,  0.805],
        [ 0.0,     0.0,     0.0,     1.0]
    ]),
    "camera_eih": np.array([
        [ 1.0,     0.0,     0.0,     0.5],
        [ 0.0,     1.0,     0.0,     0.0],
        [ 0.0,     0.0,     1.0,     0.5],
        [ 0.0,     0.0,     0.0,     1.0]
    ]), # TODO: Update this in realtime to the current camera pose.
    "camera_base": np.array([
        [ 1.0,     0.0,     0.0,     -0.5],
        [ 0.0,     1.0,     0.0,     0.0],
        [ 0.0,     0.0,     1.0,     -0.5],
        [ 0.0,     0.0,     0.0,     1.0]
    ]),
}


class ExtrinsicsTfPublisher:
    def __init__(self, camera_tf_prefix: str):
        self._camera_name = camera_tf_prefix
        self._camera_base_frame_id = f"{camera_tf_prefix}_base_link"
        self._camera_optical_frame_id = f"{camera_tf_prefix}_depth_optical_frame"
        self._robot_base_frame_id = "panda_link0"

        # TODO: Update so that the tf gui updates base_link instead of camera_depth_optical_frame. 
        # panda_link0 and  camera_depth_optical_frame shouldn't be set directly. base_link should be the one to move 
        # around. Next, print out the transform from panda_link0 to camera_depth_optical_frame for use in Mpcm2.

        # Set up TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.tf_rate = rospy.Rate(50)

        # Set up TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self._camera_base__T__optical = self.get_camera_base__T__optical()
        assert self._camera_base__T__optical is not None

        # Set up interactive marker server
        tf_seed_matrix = EXTRINSICS_MATRICES[self._camera_name]
        self._last_world__T__base_link = _transformation_matrix_to_pose(tf_seed_matrix)
        self.server = InteractiveMarkerServer(f"{self._camera_name}__extrinsics_adjuster")
        marker = make_6dof_marker(self._robot_base_frame_id, self._camera_base_frame_id, self._last_world__T__base_link)
        self.server.insert(marker, self.process_feedback)
        self.server.applyChanges()

        # Printout the transforms
        world__T__camera_optical = tf_seed_matrix @ self._camera_base__T__optical
        print(self._camera_name, f"camera_base__T__{self._camera_name}_optical", self._camera_base__T__optical)
        print(self._camera_name, f"world__T__{self._camera_name}_camera_optical", world__T__camera_optical)


    def get_camera_base__T__optical(self):
        """Get the transform from camera base frame to camera optical frame as a 4x4 matrix"""
        try:
            # Look up the transform from camera base to camera optical frame
            transform = self.tf_buffer.lookup_transform(
                self._camera_base_frame_id, 
                self._camera_optical_frame_id, 
                rospy.Time(0), 
                rospy.Duration(1.0)
            )
            # Convert TransformStamped to 4x4 matrix
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            rotation_matrix = tft.quaternion_matrix([rotation.x, rotation.y, rotation.z, rotation.w])
            rotation_matrix[0, 3] = translation.x
            rotation_matrix[1, 3] = translation.y
            rotation_matrix[2, 3] = translation.z
            return rotation_matrix
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"Could not get transform from {self._camera_base_frame_id} to {self._camera_optical_frame_id}: {e}")
            return None


    def process_feedback(self, feedback):
        pose_msg = PoseStamped()
        pose_msg.header = feedback.header
        pose_msg.pose = feedback.pose
        self._last_world__T__base_link = pose_msg.pose

        pos = self._last_world__T__base_link.position
        ori = self._last_world__T__base_link.orientation
        # Quaternion as [x, y, z, w]
        quat = [ori.x, ori.y, ori.z, ori.w]
        # 4x4 from quaternion and translation
        world__T__base_link = tft.quaternion_matrix(quat)
        world__T__base_link[0, 3] = pos.x
        world__T__base_link[1, 3] = pos.y
        world__T__base_link[2, 3] = pos.z
        world__T__camera_optical = world__T__base_link @ self._camera_base__T__optical

        # Print the transforms
        rospy.loginfo("\n___________________________________________________________________________\n")
        for T, name in zip([world__T__base_link, world__T__camera_optical, self._camera_base__T__optical], [f"world__T__{self._camera_name}_base_link", f"world__T__{self._camera_name}_camera_optical", f"camera_base__T__{self._camera_name}_optical"]):
            rospy.loginfo(f"{name}\n  --> Pose: {_transformation_matrix_to_pose(T)}\n  --> T:    {T}")

    def run(self):
        rospy.loginfo(f"Publishing transform from {self._robot_base_frame_id} to {self._camera_base_frame_id}")

        while not rospy.is_shutdown():
            # Update timestamp
            tf = TransformStamped()
            tf.header.stamp = rospy.Time.now()
            tf.header.frame_id = self._robot_base_frame_id
            tf.child_frame_id = self._camera_base_frame_id
            tf.transform.translation.x = self._last_world__T__base_link.position.x
            tf.transform.translation.y = self._last_world__T__base_link.position.y
            tf.transform.translation.z = self._last_world__T__base_link.position.z
            tf.transform.rotation.x = self._last_world__T__base_link.orientation.x
            tf.transform.rotation.y = self._last_world__T__base_link.orientation.y
            tf.transform.rotation.z = self._last_world__T__base_link.orientation.z
            tf.transform.rotation.w = self._last_world__T__base_link.orientation.w
            self.tf_broadcaster.sendTransform(tf)
            self.tf_rate.sleep()


def main():
    parser = argparse.ArgumentParser(description="Publish fixed transform between camera and robot base")
    parser.add_argument('--camera_tf_prefix', type=str, default='camera', help='Camera name (e.g., camera_south, camera_north)')
    args, unknown = parser.parse_known_args()
    assert args.camera_tf_prefix is not None
    assert any(x in args.camera_tf_prefix for x in ["south", "north", "eih", "base"]), f"Camera tf prefix not recognized: {args.camera_tf_prefix}. Unknown arguments: {unknown}"

    # Initialize node (namespace is handled by launch file's <group ns="...">)
    rospy.init_node('camera_robot_tf_publisher', anonymous=False)
    rospy.loginfo(f"Unknown arguments: {unknown}")
    publisher = ExtrinsicsTfPublisher(args.camera_tf_prefix)
    publisher.run()

if __name__ == '__main__':
    main()
