import math
import os
import shutil
import setup_path
import airsim
import pprint
import time
import utils
import models
import matplotlib.pyplot as plt
import survey_navigator
import sys
import argparse

client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)

# Initialize data lists
altitude_data_list = []
altitude_local_data_list = []
gimbal_data_list = []
timestamp_list = []
snapshot_data_list = []

# Setup live plot
plt.ion()
fig, ax_altitude = plt.subplots()
(line,) = ax_altitude.plot([], [], marker="o")

ax_altitude.set_xlabel("Time (s)")
ax_altitude.set_ylabel("Altitude (m)")
ax_altitude.set_title("Drone Altitude over Time")
ax_altitude.grid(True)

# Show the plot window immediately
plt.show(block=False)

print(
    "Press \n"
    "'S' for Survey Mission,\n"
    "'M' for Move-to-Position mode,\n"
    "'P' for Predefined Path mode."
)

choice = input("Your choice: ").lower()

survey_mode = True

# if choice == "s":
#     print("Starting Survey Navigator mission...")
#     args = sys.argv[1:]  # Remove script name from args
#     arg_parser = argparse.ArgumentParser("Usage: survey boxsize stripewidth altitude")
#     arg_parser.add_argument(
#         "--size", type=float, help="size of the box to survey", default=50
#     )
#     arg_parser.add_argument(
#         "--stripewidth",
#         type=float,
#         help="stripe width of survey (in meters)",
#         default=10,
#     )
#     arg_parser.add_argument(
#         "--altitude",
#         type=float,
#         help="altitude of survey (in positive meters)",
#         default=30,
#     )
#     arg_parser.add_argument(
#         "--speed", type=float, help="speed of survey (in meters/second)", default=5
#     )
#     parsed_args = arg_parser.parse_args(args)

#     # Import and create survey navigator
#     navigator = survey_navigator.SurveyNavigator(parsed_args)
#     navigator.start()

# elif choice == "m":
#     print("Manual move mode selected.")
#     survey_mode = False
#     airsim.wait_key("Press any key to takeoff and start altitude graphing")
#     print("Taking off...")
#     client.armDisarm(True)
#     client.takeoffAsync().join()
#     print("Drone has taken off.")
#     time.sleep(1)

#     # client.hoverAsync().join()
#     print("GPS Location Data:", utils.get_gps_location_data(client))
#     print("Local Position Data:", utils.get_local_position_data(client))

#     airsim.wait_key("Press any key to move vehicle to (20, 30, -50) at 3 m/s")
#     client.moveToPositionAsync(20, 30, -50, 3).join()
#     print("GPS Location Data:", utils.get_gps_location_data(client))
#     print("Local Position Data:", utils.get_local_position_data(client))
#     client.hoverAsync().join()

start_time = time.time()


if choice == "p":
    print("Predefined path mode selected.")
    survey_mode = False
    airsim.wait_key("Press any key to takeoff and start altitude graphing")
    print("Taking off...")
    client.armDisarm(True)

    # ✅ log altitude before movement
    initial_altitude = utils.get_local_position_data(client)[2]
    elapsed_time = time.time() - start_time
    utils.add_altitude_local_data(
        altitude_local_data_list, initial_altitude, timestamp_list, elapsed_time
    )

    client.takeoffAsync().join()
    print("Drone has taken off.")
    time.sleep(1)

    # AirSim uses NED coordinates so negative axis is up.
    # z of -15 is 15 meters above the original launch point.
    z = -30
    print("make sure we are hovering at {} meters...".format(-z))
    client.moveToZAsync(z, 1).join()
    # see https://github.com/Microsoft/AirSim/wiki/moveOnPath-demo
    # this method is async and we are not waiting for the result since we are passing timeout_sec=0.
    home_geo_point = client.getHomeGeoPoint()
    print("Home GeoPoint:", home_geo_point)
    local_position_v3 = client.getMultirotorState().kinematics_estimated.position
    print("Local Position (V3):", local_position_v3)

    print("flying on path...")
    result = client.moveOnPathAsync(
        [
            airsim.Vector3r(50, 0, z),
            airsim.Vector3r(50, -70, z),
            airsim.Vector3r(0, -70, z),
            airsim.Vector3r(0, 0, z),
        ],
        12,  # velocity
        500,  # timeout_sec
        airsim.DrivetrainType.ForwardOnly,
        airsim.YawMode(False, 0),
        20,  # lookahead
        1,  # adaptive_lookahead
    )
    # .join()

print("Movement command issued. Monitoring altitude...")

if not survey_mode:
    utils.clean_save_dir()

    flying = True
    i = 0

    # Get camera info before and after gimbal adjustment
    rotated_cam_info = utils.get_cam_info(client, camera_name="0")
    # print("Camera Info Before Gimbal:", cam_info)
    utils.set_gimbal_status(client, camera_name="0", pitch_degree=-90)
    rotated_cam_info = utils.get_cam_info(client, camera_name="0")
    print("Gimbal set to -90 degrees pitch.")
    print("Camera Info:", rotated_cam_info)

    while flying:
        multirotor_state = client.getMultirotorState()
        if (
            multirotor_state.landed_state == airsim.LandedState.Landed and i > 10
        ):  # check after a few iterations
            print("Drone has landed.")
            flying = False

        airsim_gps_data = client.getGpsData()
        gps_Data = models.GpsData(
            latitude=airsim_gps_data.gnss.geo_point.latitude,
            longitude=airsim_gps_data.gnss.geo_point.longitude,
            altitude=airsim_gps_data.gnss.geo_point.altitude,
        )

        airsim_magnetometer_data = client.getMagnetometerData()
        magnetometer_data = models.MagnetometerData(
            time_stamp=airsim_magnetometer_data.time_stamp,
            magnetic_field_body=(
                airsim_magnetometer_data.magnetic_field_body.x_val,
                airsim_magnetometer_data.magnetic_field_body.y_val,
                airsim_magnetometer_data.magnetic_field_body.z_val,
            ),
            magnetic_field_covariance=airsim_magnetometer_data.magnetic_field_covariance,
        )

        # Always get the latest camera info here
        rotated_cam_info = utils.get_cam_info(client, camera_name="0")

        drone_data_moment = models.DroneData(
            gps_data=gps_Data,
            gimbal_data=utils.quaternion_to_euler(rotated_cam_info.pose.orientation),
            snapshot_data_list=utils.take_images(client, camera_name="3"),
            magnetometer_data=magnetometer_data,
        )

        altitude_gps = drone_data_moment.gps_data.altitude
        altitude_local = (
            utils.get_local_position_data(client)[2] * -1
        )  # Convert to positive altitude
        elapsed_time = time.time() - start_time

        # For Drawing Graphic
        # altitude_data_list.append(altitude)
        utils.add_altitude_local_data(
            altitude_local_data_list, altitude_local, timestamp_list, elapsed_time
        )

        # Update line data instead of clearing the plot
        utils.update_line_data(
            line, ax_altitude, fig, timestamp_list, altitude_local_data_list
        )
        rotated_cam_info = utils.get_cam_info(client, camera_name="0")
        print("Rotated Camera Info:", rotated_cam_info)

        print(f"Time: {elapsed_time:.2f}s, Altitude Local: {altitude_local:.2f}m")
        print("----- Drone Data Count -----", len(drone_data_moment))
        # pprint.pprint(drone_data_moment)

        # Stop after timeout
        if elapsed_time > 500:  # Increased timeout to allow for movement
            print("Timeout reached — stopping.")
            flying = False

        time.sleep(1)
        i += 1

    print("Resetting to original state...")

# drone will over-shoot so we bring it back to the start point before landing.
client.moveToPositionAsync(0, 0, z, 1).join()

print("landing...")
client.landAsync().join()

print("disarming...")
client.armDisarm(False)

client.enableApiControl(False)
print("done.")

plt.ioff()
plt.show()
