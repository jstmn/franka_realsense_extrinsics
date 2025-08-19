#!/home/jstm/Projects/mpcm2/deoxys_venv/bin/python3
import argparse
import rospy
from realsense2_camera.msg import Extrinsics
from sensor_msgs.msg import JointState, Image, CameraInfo
from deoxys.franka_interface import FrankaInterface
import numpy as np
from pathlib import Path
import h5py
from datetime import datetime
import os
import json
import cv2

# $ rostopic list
# ...
#  * /realsense/depth/image_rect_raw [sensor_msgs/Image] 1 publisher
#  * /realsense/depth/color/points [sensor_msgs/PointCloud2] 1 publisher
#  * /realsense/color/image_raw [sensor_msgs/Image] 1 publisher
#  * /realsense/color/camera_info [sensor_msgs/CameraInfo] 1 publisher
#  * /realsense/depth/camera_info [sensor_msgs/CameraInfo] 1 publisher
#  * /realsense/extrinsics/depth_to_color [realsense2_camera/Extrinsics] 1 publisher
#  * /panda/joint_states [sensor_msgs/JointState] 1 publisher
# ...

def to_public_dict(obj):
    """Recursively convert an object to a dict, skipping private attributes."""
    # Base cases: primitives
    if isinstance(obj, (str, int, float, bool, type(None))):
        return obj
    if isinstance(obj, (list, tuple)):
        return [to_public_dict(item) for item in obj]
    if isinstance(obj, dict):
        return {k: to_public_dict(v) for k, v in obj.items()}
    if hasattr(obj, "__dict__"):
        return {k: to_public_dict(v)
                for k, v in obj.__dict__.items()
                if not k.startswith("_")}
    if hasattr(obj, "__slots__"):
        return {slot: to_public_dict(getattr(obj, slot))
                for slot in obj.__slots__
                if hasattr(obj, slot) and not slot.startswith("_")}
    # Fallback to dir()
    result = {}
    for attr in dir(obj):
        if attr.startswith("_"):
            continue
        value = getattr(obj, attr)
        if callable(value):
            continue
        result[attr] = to_public_dict(value)
    return result


class DemonstrationLogger:
    def __init__(self, parent_output_dir: Path, description: str, rate: int = 15):
        self._description = description
        self._tstart = datetime.now().strftime("%m-%d_%H:%M:%S")
        assert parent_output_dir.exists(), f"The provided parent output directory: '{parent_output_dir}' does not exist"
        if len(description) > 0:
            self._output_dir = parent_output_dir / f"{self._tstart}__{self._description}"
        else:
            self._output_dir = parent_output_dir / str(self._tstart)
        self._output_dir.mkdir(parents=True, exist_ok=True)
        self.rate = rospy.Rate(rate)  # 10 Hz

        # Create a subscriber to the depth image
        self.depth_sub = rospy.Subscriber("/realsense/depth/image_rect_raw", Image, self.depth_callback)
        self.color_sub = rospy.Subscriber("/realsense/color/image_raw", Image, self.color_callback)
        self.depth_camera_info_sub = rospy.Subscriber("/realsense/depth/camera_info", CameraInfo, self.depth_camera_info_callback)
        self.color_camera_info_sub = rospy.Subscriber("/realsense/color/camera_info", CameraInfo, self.color_camera_info_callback)
        self.extrinsics_sub = rospy.Subscriber("/realsense/extrinsics/depth_to_color", Extrinsics, self.extrinsics_callback)
        self.joint_states_sub = rospy.Subscriber("/panda/joint_states", JointState, self.joint_states_callback)

        self._depth_image = None
        self._color_image = None
        self._depth_camera_info = None
        self._color_camera_info = None
        self._joint_states = None
        self._extrinsics = None
        self._depth_image_msgs = []
        self._color_image_msgs = []
        self._joint_states_msgs = []
        rospy.on_shutdown(self.save_data)

    def depth_callback(self, msg: Image):
        self._depth_image = msg

    def color_callback(self, msg: Image):
        self._color_image = msg

    def extrinsics_callback(self, msg: Extrinsics):
        self._extrinsics = msg

    def depth_camera_info_callback(self, msg: CameraInfo):
        self._depth_camera_info = msg

    def color_camera_info_callback(self, msg: CameraInfo):
        self._color_camera_info = msg

    def joint_states_callback(self, msg: JointState):
        self._joint_states = msg

    def save_data(self):
        if len(self._depth_image_msgs) == 0:
            rospy.logwarn("Skipping data saving because no data was received")
            return

        h5_filepath = os.path.join(self._output_dir, f"data.h5")
        with h5py.File(h5_filepath, "w") as f:
            depth_images = np.array([np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width) for msg in self._depth_image_msgs])
            color_images = np.array([np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1) for msg in self._color_image_msgs])
            q = np.array([msg.position for msg in self._joint_states_msgs])
            dq = np.array([msg.velocity for msg in self._joint_states_msgs])
            print("\n==============================")
            print("Data shapes:")
            print(f"  depth_images: {depth_images.shape}")
            print(f"  color_images: {color_images.shape}")
            print(f"  q:            {q.shape}")
            print(f"  dq:           {dq.shape}")
            print()
            assert depth_images.shape[0] == color_images.shape[0] == q.shape[0] == dq.shape[0]
            f.create_dataset("depth_image", data=depth_images)
            f.create_dataset("color_image", data=color_images)
            f.create_dataset("joint_states_q", data=q)
            f.create_dataset("joint_states_dq", data=dq)


        for name, msg in [("camera_info_depth", self._depth_camera_info), ("camera_info_color", self._color_camera_info), ("camera_internal_extrinsics", self._extrinsics)]:
            with open(os.path.join(self._output_dir, f"{name}.json"), "w") as f:
                json.dump(to_public_dict(msg), f)

        # Save some images for debugging
        img_dir = os.path.join(self._output_dir, "images")
        os.makedirs(img_dir, exist_ok=True)
        for i in range(depth_images.shape[0]):
            cv2.imwrite(os.path.join(img_dir, f"depth_{i}.png"), depth_images[i])
            cv2.imwrite(os.path.join(img_dir, f"color_{i}.png"), color_images[i])

        rospy.loginfo(f"Data saved to {h5_filepath}")

    def run(self):
        counter = 0
        empty_counter = 0
        while not rospy.is_shutdown():
            self.rate.sleep()
            assert len(self._depth_image_msgs) == len(self._color_image_msgs) == len(self._joint_states_msgs), f""

            if any(msg is None for msg in [self._depth_image, self._color_image, self._depth_camera_info, self._color_camera_info, self._joint_states]):
                empty_counter += 1
                if empty_counter % 10 == 0:
                    rospy.loginfo(f"No data received for {empty_counter} samples")
                if empty_counter > 50:
                    rospy.logerr(f"No data received for {empty_counter} samples, shutting down")
                    rospy.signal_shutdown("No data received")
                continue

            # TODO: Check if objects are being updated

            # Save data
            self._depth_image_msgs.append(self._depth_image)
            self._color_image_msgs.append(self._color_image)
            self._joint_states_msgs.append(self._joint_states)

            if counter % 50 == 0:
                rospy.loginfo(f"Logging data... ({len(self._joint_states_msgs)} samples)")
            counter += 1



def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output_dir", type=str)
    parser.add_argument("--description", type=str)
    args, _ = parser.parse_known_args()
    rospy.init_node('log_demonstration', anonymous=False)
    logger = DemonstrationLogger(Path(args.output_dir), args.description)
    logger.run()

if __name__ == '__main__':
    main()
