#!/usr/bin/env python
############################################################################
#
#   Copyright (C) 2022 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

__author__ = "Jaeyoung Lim"
__contact__ = "jalim@ethz.ch"

import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.task import Future

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus

import asyncio
from mavsdk import System, offboard, telemetry
import utils


class OffboardControl(Node):

    def __init__(self):
        # INITIALIZE ROS NODES AND CONNECTIONS
        super().__init__('minimal_publisher')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile)
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.dt = timer_period
        self.theta = 0.0
        self.radius = 10.0
        self.omega = 0.5

        # INITIALIZE MAVLINK CONNECTIONS (don't do unless have to incase this reduces ROS' communication bandwidth)


    def vehicle_status_callback(self, msg):
        # TODO: handle NED->ENU transformation
        print("NAV_STATUS: ", msg.nav_state)
        print("  - offboard status: ", VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        self.nav_state = msg.nav_state

    def cmdloop_callback(self):
        # Publish offboard control modes
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position=True
        offboard_msg.velocity=False
        offboard_msg.acceleration=False
        self.publisher_offboard_mode.publish(offboard_msg)
        if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:

            trajectory_msg = TrajectorySetpoint()
            trajectory_msg.position[0] = self.radius * np.cos(self.theta)
            trajectory_msg.position[1] = self.radius * np.sin(self.theta)
            trajectory_msg.position[2] = -5.0
            self.publisher_trajectory.publish(trajectory_msg)

            self.theta = self.theta + self.omega * self.dt



# Return system object representing drone connected on input address
# Perform health checks upon connection if desired
async def wait_for_drone(drone):
    print("Waiting for drone...")

    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"Drone discovered") #with UUID: {state.uuid}")
            break

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break
    
    print("Drone successfully connected \n")

# Arm, takeoff and switch to offboard control mode using MAVSDK (can alternatively do on remote)
# TODO: Add error checking between connection, arming and each mode transition to ensure successful. See: https://mavsdk.mavlink.io/main/en/cpp/guide/taking_off_landing.html 
async def mission_start(drone):
    print("STARTING: Takeoff routine")

    # Start in hold mode
    await drone.action.hold()

    # Arm drone and wait 2 sec
    print("-- Arming")
    await drone.action.arm()
    await asyncio.sleep(2)

    # Get drone to take off
    print("-- Taking off")
    await drone.action.set_takeoff_altitude(2)
    await drone.action.takeoff()

    # Wait until takeoff complete
    async for current_flight_mode in drone.telemetry.flight_mode(): 
        if current_flight_mode == telemetry.FlightMode.HOLD:
            break

    await asyncio.sleep(2)

    print("COMPLETE: Takeoff routine \n")


# Use MAVLink to send waypoints
async def mission_offboard(drone):
    print("STARTING: Offboard routine")
    
    # Start sending velocity command (stay at current position)
    vel_start = offboard.VelocityBodyYawspeed(0,0,0,0)
    await drone.offboard.set_velocity_body(vel_start)

    # Switch to offboard control mode
    print("-- Switch to offboard control")
    await drone.offboard.start()

    # Fly forward for 10 sec
    # print("-- Flying at set velocity")
    # vel_2 = offboard.VelocityBodyYawspeed(2,0,0,0)
    # await drone.offboard.set_velocity_body(vel_start)
    # await asyncio.sleep(10)

    # Fly to desired position
    print("-- Flying to set position")
    vel_2 = offboard.VelocityNedYaw(0.25,0,0,0)
    pos_2 = offboard.PositionNedYaw(10,0,-2,0)
    await drone.offboard.set_position_velocity_ned(pos_2, vel_2)

    # Only finish flight when within desired error of setpoint (might need to remove velocity feed-forward for more precise positioning)
    pos_2_set = telemetry.PositionNed(pos_2.north_m, pos_2.east_m, pos_2.down_m)
    err_rad = 0.25

    async for drone_pos_vel in drone.telemetry.position_velocity_ned():
        # print(f"Current: {drone_pos_vel.position.north_m}, {drone_pos_vel.position.east_m}, {drone_pos_vel.position.down_m}")
        # print(f"Setpoint: {pos_2_set.north_m}, {pos_2_set.east_m}, {pos_2_set.down_m}")

        pos_current = (drone_pos_vel.position.north_m, drone_pos_vel.position.east_m, drone_pos_vel.position.down_m)
        pos_setpoint = (pos_2_set.north_m, pos_2_set.east_m, pos_2_set.down_m)

        if utils.within_radius_3D(pos_current, pos_setpoint, err_rad):
            break
    
    print("-- At desired position")
    await asyncio.sleep(5)
    print("COMPLETE: offboard routine \n")



# RTL, land and disarm using MAVSDK (can alternatively do on remote)
# TODO: Error checking
async def mission_end(drone):
    print("STARTING: Landing routine")

    # Turn off offboard mode
    await drone.offboard.stop()

    # Return to home and land
    print("-- Returning to home")
    await drone.action.set_return_to_launch_altitude(5)
    await drone.action.return_to_launch()

    # Only disarm once landed
    async for drone_state in drone.telemetry.landed_state():
        if drone_state == telemetry.LandedState.ON_GROUND:
            break
    await drone.action.disarm()
    #await drone.action.hold()

    print("COMPLETE: Landing routine \n")


async def run_mavlink():
    # Connect to drone via MAVLINK
    drone = System()
    system_address="udp://:14540"
    await drone.connect(system_address)
    await wait_for_drone(drone)

    # Start mission (maybe replace entirely with offboard later - for multi-drone co-ordination)
    await mission_start(drone)

    # Run mission
    await mission_offboard(drone)

    # End mission (maybe replace entirely with offboard later - for multi-drone co-ordination)
    await mission_end(drone)


def main(args=None):
    # Perform offboard control with MAVSDK
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run_mavlink())

    # Perform offboard control with ROS2
    # (note required to run in a different terminal: micro-ros-agent udp4 --port 8888)
    # rclpy.init(args=args)
    # offboard_control = OffboardControl()

    # future_msg = Future() # Perform onboard control until a msg is returned (never happens currently) or a timeout is reached
    # rclpy.spin_until_future_complete(offboard_control, future_msg, timeout_sec=10) #rclpy.spin(offboard_control)

    # offboard_control.destroy_node()
    # rclpy.shutdown()

if __name__ == '__main__':
    main()
