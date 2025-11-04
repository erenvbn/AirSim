from http import client
import math
import shutil
import numpy as np
import os
import tempfile
import pprint
import cv2
import time
import airsim
import models


def inspectClass(obj):
    print("Type: {}".format(type(obj)))
    print("Attributes:")
    for attr in dir(obj):
        if not attr.startswith("__"):
            print(" - {}: {}".format(attr, getattr(obj, attr)))


def get_gps_location_data(client: airsim.MultirotorClient):
    home_geo_point = client.getHomeGeoPoint()
    return home_geo_point
    # print("Home GeoPoint:", home_geo_point)


def vector3r_to_tuple(v: airsim.Vector3r):
    """Convert AirSim Vector3r to a tuple (x, y, z)."""
    return (v.x_val, v.y_val, v.z_val)


def get_local_position_data(client: airsim.MultirotorClient):
    """Return local position as a tuple (x, y, z) to allow indexing."""
    local_position_v3 = client.getMultirotorState().kinematics_estimated.position
    return vector3r_to_tuple(local_position_v3)
    # print("Local Position vector3:", local_position_v     3)


def get_location_data(client: airsim.MultirotorClient):
    home_geo_point = get_gps_location_data(client)
    local_position_v3 = get_local_position_data(client)
    print("Home GeoPoint:", home_geo_point)
    print("Local Position vector3:", local_position_v3)


def add_altitude_local_data(
    altitude_local_data_list, initial_altitude, timestamp_list, elapsed_time
):
    altitude_local_data_list.append(initial_altitude)
    timestamp_list.append(elapsed_time)


def update_line_data(line, ax_altitude, fig, timestamp_list, altitude_local_data_list):
    line.set_xdata(timestamp_list)
    line.set_ydata(altitude_local_data_list)
    ax_altitude.relim()
    ax_altitude.autoscale_view()
    fig.canvas.draw()
    fig.canvas.flush_events()


def get_save_dir():
    # Get the directory where the current script (utils.py) is located
    script_dir = os.path.dirname(os.path.abspath(__file__))
    save_dir = os.path.join(script_dir, "airsim_drone_images")
    return save_dir


def take_images(client: airsim.MultirotorClient, camera_name):
    # The following line is where the images are **taken** from the simulator.
    # It requests four images of different types from different cameras.

    # Camera ids: 0,1,2,3,4
    # front_center, front_right, front_left, fpv and back_center.
    # https://github.com/Microsoft/AirSim/blob/main/docs/image_apis.md#multirotor
    snapshot_data_list = []
    responses = client.simGetImages(
        [
            # airsim.ImageRequest(
            #     camera_name, airsim.ImageType.DepthVis
            # ),  # depth visualization image
            # airsim.ImageRequest(
            #     camera_name, airsim.ImageType.DepthPerspective, True
            # ),  # depth in perspective projection
            airsim.ImageRequest(
                camera_name, airsim.ImageType.Scene
            ),  # scene vision image in png format
            # airsim.ImageRequest(f"{camera_name}", airsim.ImageType.Scene, False, False),
        ]
    )  # scene vision image in uncompressed RGBA array
    # print("Retrieved images: %d" % len(responses))

    save_dir = get_save_dir()

    # print(f"Saving images to {save_dir}")

    # timestamp_dir = os.path.join(save_dir, time.strftime("%Y%m%d_%H%M%S"))
    epoch_time = int(time.time() * 1000)  # milliseconds since epoch

    # timestamp_dir = os.path.join(save_dir, str(epoch_time))
    # os.makedirs(timestamp_dir, exist_ok=True)

    # The following loop is where the captured images are **saved** to files.
    image_index = 0
    for response in responses:
        snapshot_data = models.SnapshotData(
            timestamp=epoch_time,
            imagePath="",
            image_type=0,
            width=0,
            height=0,
        )

        filename = os.path.join(save_dir, f"{epoch_time}_{image_index}")

        # filename = os.path.join(timestamp_dir, str(image_index))

        # It checks the image format and uses the appropriate function to write the file.
        # If the image is in float format (e.g., depth), it saves it as a PFM file.
        if response.pixels_as_float:
            # print(
            #     "Type %d, size %d, fileName: %s"
            #     % (
            #         response.image_type,
            #         len(response.image_data_float),
            #         filename + ".pfm",
            #     )
            # )
            airsim.write_pfm(
                os.path.normpath(filename + ".pfm"), airsim.get_pfm_array(response)
            )
            snapshot_data.imagePath = os.path.normpath(filename + ".pfm")
            snapshot_data.image_data_uint8 = airsim.get_pfm_array(response).tobytes()
            snapshot_data_list.append(snapshot_data)

        # If the image is compressed (e.g., PNG), it saves it directly.
        elif response.compress:  # png format
            # print(
            #     "Type %d, size %d, fileName: %s"
            #     % (
            #         response.image_type,
            #         len(response.image_data_uint8),
            #         filename + ".png",
            #     )
            # )
            airsim.write_file(
                os.path.normpath(filename + ".png"), response.image_data_uint8
            )

            snapshot_data.imagePath = os.path.normpath(filename + ".png")
            snapshot_data.image_data_uint8 = response.image_data_uint8
            snapshot_data_list.append(snapshot_data)

            # If the image is uncompressed, it saves it as a PNG file.
        else:  # uncompressed array
            # print(
            #     "Type %d, size %d, fileName: %s"
            #     % (
            #         response.image_type,
            #         len(response.image_data_uint8),
            #         filename + ".png",
            #     )
            # )
            img1d = np.fromstring(
                response.image_data_uint8, dtype=np.uint8
            )  # get numpy array
            img_rgb = img1d.reshape(
                response.height, response.width, 3
            )  # reshape array to 4 channel image array H X W X 3
            cv2.imwrite(os.path.normpath(filename + ".png"), img_rgb)  # write to png
            # Add created image paths to snapshot_data_list
            snapshot_data.imagePath = os.path.normpath(filename + ".png")
            snapshot_data.image_data_uint8 = response.image_data_uint8
            snapshot_data_list.append(snapshot_data)

        image_index += 1

    return snapshot_data_list


def clean_save_dir():
    save_dir = get_save_dir()
    if os.path.exists(save_dir):
        shutil.rmtree(save_dir)
        print(f"Removed existing directory: {save_dir}")

    os.makedirs(save_dir, exist_ok=True)

    # print("Saving images to %s" % save_dir)
    try:
        os.makedirs(save_dir)
    except OSError:
        if not os.path.isdir(save_dir):
            raise


def quaternion_to_euler(q):
    """Convert AirSim quaternion to Euler angles in degrees"""
    w, x, y, z = q.w_val, q.x_val, q.y_val, q.z_val

    # roll (x-axis)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis)
    sinp = 2 * (w * y - z * x)
    pitch = math.asin(max(-1, min(1, sinp)))

    # yaw (z-axis)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    gimbal_data = models.GimbalData(
        pitch=math.degrees(pitch), yaw=math.degrees(yaw), roll=math.degrees(roll)
    )

    # convert to degrees
    return gimbal_data


# Quaternion helpers and camera rotation utilities
def _quat_mul(q1: airsim.Quaternionr, q2: airsim.Quaternionr) -> airsim.Quaternionr:
    """Hamilton product q = q1 ⊗ q2"""
    w1, x1, y1, z1 = q1.w_val, q1.x_val, q1.y_val, q1.z_val
    w2, x2, y2, z2 = q2.w_val, q2.x_val, q2.y_val, q2.z_val
    q = airsim.Quaternionr()
    q.w_val = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    q.x_val = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    q.y_val = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    q.z_val = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return q


def set_camera_pitch_relative(
    client: airsim.MultirotorClient, camera_name="0", delta_pitch_degree=0
):
    """Rotate camera around its local X-axis by delta_pitch_degree."""
    info = client.simGetCameraInfo(camera_name)
    q_cur = info.pose.orientation
    q_delta = airsim.to_quaternion(
        math.radians(delta_pitch_degree), 0, 0
    )  # local X (pitch)
    q_new = _quat_mul(q_cur, q_delta)  # apply in camera local frame
    client.simSetCameraPose(camera_name, airsim.Pose(info.pose.position, q_new))
    return client.simGetCameraInfo(camera_name)


def get_camera_euler(client: airsim.MultirotorClient, camera_name="0"):
    """Return GimbalData(pitch,yaw,roll) for a camera."""
    info = client.simGetCameraInfo(camera_name)
    return quaternion_to_euler(info.pose.orientation)


def set_gimbal_status(
    client: airsim.MultirotorClient, camera_name="0", pitch_degree=15
):
    # Gimbal adjustment code (optional)
    # airsim.wait_key("Press any key to set camera-0 gimbal to 15-degree pitch")
    # sets the pose for the specified camera while taking an
    # input pose as a combination of relative position and a quaternion in NED frame.
    # The handy `airsim.to_quaternion()` function allows to convert pitch, roll, yaw to quaternion.
    # For example, to set camera-0 to 15-degree pitch while maintaining the same position, you can use:

    camera_pose = airsim.Pose(
        airsim.Vector3r(0, 0, 0), airsim.to_quaternion(math.radians(pitch_degree), 0, 0)
    )  # radians

    client.simSetCameraPose(camera_name, camera_pose)
    # return cam_info
    return client.simGetCameraInfo(camera_name)


def get_cam_info(client: airsim.MultirotorClient, camera_name="0"):
    cam_info = client.simGetCameraInfo(camera_name)
    # gimbal_data = quaternion_to_euler(cam_info.pose.orientation)
    return cam_info


def is_ok_gimbal(pitch, max_pitch, min_pitch):
    # roll can be near 0° or near 180° depending on AirSim gimbal orientation
    return abs(pitch) <= max_pitch and abs(pitch) >= min_pitch


def update_scatter_data(scatter_objs, ax, fig, datasets):
    """
    scatter_objs: list of scatter plot objects
    datasets: list of (x_list, y_list) tuples, same order as scatter_objs
    """
    combined_pts = []

    for scatter, (x_data, y_data) in zip(scatter_objs, datasets):
        if len(x_data) and len(y_data):
            pts = np.column_stack((x_data, y_data))
            scatter.set_offsets(pts)
            combined_pts.append(pts)
        else:
            scatter.set_offsets(np.empty((0, 2)))

    # relim() ignores scatter; update limits from points explicitly
    if combined_pts:
        all_pts = np.vstack(combined_pts)
        ax.update_datalim(all_pts)
        ax.autoscale_view()

    fig.canvas.draw_idle()
    fig.canvas.flush_events()
