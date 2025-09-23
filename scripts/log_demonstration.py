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
#  * /realsense_south/depth/image_rect_raw [sensor_msgs/Image] 1 publisher
#  * /realsense_south/depth/color/points [sensor_msgs/PointCloud2] 1 publisher
#  * /realsense_south/color/image_raw [sensor_msgs/Image] 1 publisher
#  * /realsense_south/color/camera_info [sensor_msgs/CameraInfo] 1 publisher
#  * /realsense_south/depth/camera_info [sensor_msgs/CameraInfo] 1 publisher
#  * /realsense_south/extrinsics/depth_to_color [realsense2_camera/Extrinsics] 1 publisher
#  * ...
#  * /realsense_north/depth/image_rect_raw [sensor_msgs/Image] 1 publisher
#  * /realsense_north/depth/color/points [sensor_msgs/PointCloud2] 1 publisher
#  * /realsense_north/color/image_raw [sensor_msgs/Image] 1 publisher
#  * /realsense_north/color/camera_info [sensor_msgs/CameraInfo] 1 publisher
#  * /realsense_north/depth/camera_info [sensor_msgs/CameraInfo] 1 publisher
#  * /realsense_north/extrinsics/depth_to_color [realsense2_camera/Extrinsics] 1 publisher
#  * ...
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


DATA_TYPES = ["depth_image", "color_image", "depth_camera_info", "color_camera_info"]

class DemonstrationLogger:
    def __init__(self, parent_output_dir: Path, description: str, rate: float, camera_suffixes: list = None):
        self._description = description
        self._tstart = datetime.now().strftime("%m-%d_%H:%M:%S")
        assert parent_output_dir.exists(), f"The provided parent output directory: '{parent_output_dir}' does not exist"
        if len(description) > 0:
            self._output_dir = parent_output_dir / f"{self._tstart}__{self._description}"
        else:
            self._output_dir = parent_output_dir / str(self._tstart)
        self._output_dir.mkdir(parents=True, exist_ok=True)
        self.rate = rospy.Rate(rate)  # 10 Hz

        # Default to single camera if no suffixes provided
        if camera_suffixes is None:
            camera_suffixes = [""]

        self._camera_suffixes = camera_suffixes
        self._subscribers = []
        self._joint_states_sub = rospy.Subscriber("/panda/joint_states", JointState, self.joint_states_callback)

        # Create subscribers for each camera
        for suffix in self._camera_suffixes:
            print()
            print('Suffix: ', suffix)
            topic_prefix = f"/realsense{suffix}"

            # Create callback functions for this specific camera
            def make_callback(suffix_, data_type: str):
                assert data_type in DATA_TYPES, f"Invalid data type: {data_type}"
                def callback(msg):
                    setattr(self, f"{suffix_}_{data_type}", msg)
                return callback

            # Create subscribers for this camera
            print('  subscribing to: ', f"{topic_prefix}/depth/image_rect_raw")
            print('  subscribing to: ', f"{topic_prefix}/color/image_raw")
            print('  subscribing to: ', f"{topic_prefix}/color/camera_info")
            print('  subscribing to: ', f"{topic_prefix}/depth/camera_info")
            depth_sub = rospy.Subscriber(f"{topic_prefix}/depth/image_rect_raw", Image, make_callback(suffix, "depth_image"))
            color_sub = rospy.Subscriber(f"{topic_prefix}/color/image_raw", Image, make_callback(suffix, "color_image"))
            depth_camera_info_sub = rospy.Subscriber(f"{topic_prefix}/depth/camera_info", CameraInfo, make_callback(suffix, "depth_camera_info"))
            color_camera_info_sub = rospy.Subscriber(f"{topic_prefix}/color/camera_info", CameraInfo, make_callback(suffix, "color_camera_info"))
            self._subscribers.extend([depth_sub, color_sub, depth_camera_info_sub, color_camera_info_sub])


        # Initialize cache
        self._joint_states = None
        self._joint_states_msgs = []
        for suffix in self._camera_suffixes:
            for data_type in DATA_TYPES:
                setattr(self, f"{suffix}_{data_type}", None)
                setattr(self, f"{suffix}_{data_type}_msgs", [])

        rospy.on_shutdown(self.save_data)

    def joint_states_callback(self, msg: JointState):
        self._joint_states = msg

    def save_data(self):
        # Check if we have any data from any camera
        has_data = False
        for suffix in self._camera_suffixes:
            depth_msgs = getattr(self, f"{suffix}_depth_image_msgs")
            if len(depth_msgs) > 0:
                has_data = True
                break

        if not has_data:
            rospy.logwarn("Skipping data saving because no data was received")
            return

        h5_filepath = os.path.join(self._output_dir, f"data.h5")
        with h5py.File(h5_filepath, "w") as f:
            # Save joint states (same for all cameras)
            q = np.array([msg.position for msg in self._joint_states_msgs])
            dq = np.array([msg.velocity for msg in self._joint_states_msgs])
            f.create_dataset("joint_states_q", data=q)
            f.create_dataset("joint_states_dq", data=dq)

            print("\n==============================")
            print("Data shapes:")
            print(f"  q:            {q.shape}")
            print(f"  dq:           {dq.shape}")

            # Save data for each camera
            for suffix in self._camera_suffixes:
                depth_msgs = getattr(self, f"{suffix}_depth_image_msgs")
                color_msgs = getattr(self, f"{suffix}_color_image_msgs")

                if len(depth_msgs) > 0:
                    depth_images = np.array([np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width) for msg in depth_msgs])
                    color_images = np.array([np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1) for msg in color_msgs])

                    # Create dataset names with camera suffix
                    dataset_prefix = f"camera_{suffix}" if suffix else "camera"
                    f.create_dataset(f"{dataset_prefix}_depth_image", data=depth_images)
                    f.create_dataset(f"{dataset_prefix}_color_image", data=color_images)
                    print(f"  {dataset_prefix}_depth_images: {depth_images.shape}")
                    print(f"  {dataset_prefix}_color_images: {color_images.shape}")
                    assert depth_images.shape[0] == color_images.shape[0] == q.shape[0] == dq.shape[0]

        # Save camera info and extrinsics for each camera
        for suffix in self._camera_suffixes:
            depth_camera_info = getattr(self, f"{suffix}_depth_camera_info")
            color_camera_info = getattr(self, f"{suffix}_color_camera_info")
            if depth_camera_info is not None:
                file_prefix = f"camera_{suffix}_" if suffix else "camera_"
                for name, msg in [(f"{file_prefix}camera_info_depth", depth_camera_info), 
                                (f"{file_prefix}camera_info_color", color_camera_info), 
                                ]:
                    if msg is not None:
                        with open(os.path.join(self._output_dir, f"{name}.json"), "w") as f:
                            json.dump(to_public_dict(msg), f)

        # Save some images for debugging
        img_dir = os.path.join(self._output_dir, "images")
        os.makedirs(img_dir, exist_ok=True)
        for suffix in self._camera_suffixes:
            depth_msgs = getattr(self, f"{suffix}_depth_image_msgs")
            color_msgs = getattr(self, f"{suffix}_color_image_msgs")
            if len(depth_msgs) > 0:
                depth_images = np.array([np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width) for msg in depth_msgs])
                color_images = np.array([np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1) for msg in color_msgs])
                img_prefix = f"camera_{suffix}_" if suffix else "camera_"
                for i in range(depth_images.shape[0]):
                    cv2.imwrite(os.path.join(img_dir, f"{img_prefix}depth_{i}.png"), depth_images[i])
                    cv2.imwrite(os.path.join(img_dir, f"{img_prefix}color_{i}.png"), color_images[i])

        rospy.loginfo(f"Data saved to {h5_filepath}")

    def run(self):
        counter = 0
        empty_counter = 0
        while not rospy.is_shutdown():
            self.rate.sleep()
            
            # Check that all camera message lists have the same length
            msg_lengths = []
            for suffix in self._camera_suffixes:
                depth_msgs = getattr(self, f"{suffix}_depth_image_msgs")
                color_msgs = getattr(self, f"{suffix}_color_image_msgs")
                msg_lengths.extend([len(depth_msgs), len(color_msgs)])
            msg_lengths.append(len(self._joint_states_msgs))
            assert len(set(msg_lengths)) <= 1, f"Message lengths mismatch: {msg_lengths}"

            # Check if we have data from all cameras and joint states
            all_data_available = True
            missing_data_names = []
            for suffix in self._camera_suffixes:
                for data_type in DATA_TYPES:
                    if getattr(self, f"{suffix}_{data_type}") is None:
                        missing_data_names.append(f"{suffix}_{data_type}")

                if len(missing_data_names) > 0:
                    all_data_available = False
                    break

            if self._joint_states is None:
                all_data_available = False
                missing_data_names.append("_joint_states")

            if not all_data_available:
                empty_counter += 1
                if empty_counter % 10 == 0:
                    rospy.loginfo(f"No data received for {empty_counter} samples: {missing_data_names}")
                if empty_counter > 50:
                    rospy.logerr(f"No data received for {empty_counter} samples, shutting down")
                    rospy.signal_shutdown("No data received")
                continue

            # Save data for all cameras
            for suffix in self._camera_suffixes:
                depth_image = getattr(self, f"{suffix}_depth_image")
                color_image = getattr(self, f"{suffix}_color_image")
                depth_msgs = getattr(self, f"{suffix}_depth_image_msgs")
                color_msgs = getattr(self, f"{suffix}_color_image_msgs")
                depth_msgs.append(depth_image)
                color_msgs.append(color_image)

            self._joint_states_msgs.append(self._joint_states)

            if counter % 50 == 0:
                rospy.loginfo(f"Logging data... ({len(self._joint_states_msgs)} samples)")
            counter += 1



def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output_dir", type=str)
    parser.add_argument("--description", type=str)
    parser.add_argument("--rate", type=float)
    parser.add_argument("--camera_suffixes", type=str, nargs="*", default=None, help="List of camera suffixes (e.g., _north _south)")
    args, unknown = parser.parse_known_args()
    print(f"Unknown arguments: {unknown}", flush=True)

    rospy.init_node('log_demonstration', anonymous=False)
    logger = DemonstrationLogger(Path(args.output_dir), args.description, args.rate, args.camera_suffixes)
    logger.run()

if __name__ == '__main__':
    main()
