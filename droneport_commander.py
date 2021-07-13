#!/usr/bin/env python
# coding: utf-8

# DronePort Mission Control with MAVLink

# Functions use pymavlink package and they are built upon examples from the package.
# Functions were tested with Python 3.8.8 64-bit on Ubuntu 20.04. 

from pymavlink import mavutil, mavwp
from geopy.distance import distance

import time
import pickle


def connect(connection_string):
    """Establish connection with drone using MAVLink.

    Input:
        connection_string: string with IP address and PORT in format 'IP:PORT'
    Output:
        mav: pymavlink object used for communication with drone
    """
    mav = mavutil.mavlink_connection('udp:'+connection_string)
    mav.wait_heartbeat()
    print(f"Connection to {connection_string} established")
    return mav


def download_mission(mav, outfile_name):
    """Download and save drone mission.

    Input:
        mav: pymavlink object with established connection
        outfile_name: string with the name of file
    """
    mav.wait_heartbeat()
    # Read Waypoint from autopilot
    mav.waypoint_request_list_send()
    waypoint_count = 0
    waypoint_current = 0

    # Read count of waypoints
    msg = mav.recv_match(type=['MISSION_COUNT'], blocking=True)
    waypoint_count = 0
    waypoint_count = msg.count

    # Read current waypoint index
    msg = mav.recv_match(type=['MISSION_CURRENT'], blocking=True)
    waypoint_current = msg.seq

    print(f"Mission Progress: {waypoint_current}/{waypoint_count}")

    # Read mission items
    mission_items = []
    for i in range(waypoint_count):
        mav.waypoint_request_send(i)
        msg = mav.recv_match(type=['MISSION_ITEM'], blocking=True)
        print('Receving waypoint {0}'.format(msg.seq))
        mission_items.append(msg)

    # OKAY
    mav.mav.mission_ack_send(mav.target_system, mav.target_component, 0)

    # Save mission to file using pickle
    with open(outfile_name, 'wb') as outfile:
        pickle.dump([waypoint_current, waypoint_count, mission_items],
                    outfile, pickle.HIGHEST_PROTOCOL)

    return waypoint_current


def upload_mission(mav, mission_items):
    """Upload mission to drone.

    Input:
        mav: pymavlink object with established connection
        mission_items: array of tupples with coordinates (latitude, longitude, altitude) in format 
                        [(lat0,lon0,alt0), (lat1,lon1,alt1),...]
    """
    wp = mavwp.MAVWPLoader()

    seq = 0
    for waypoint in enumerate(mission_items):
        frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
        seq = waypoint[0]
        lat, lon, altitude = waypoint[1]
        autocontinue = 1
        current = 0
        param1 = 15.0  # minimum pitch
        if seq == 0:  # first waypoint to takeoff
            current = 1
            p = mavutil.mavlink.MAVLink_mission_item_message(
                mav.target_system, mav.target_component, seq, frame, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, current, autocontinue, param1, 0, 0, 0, lat, lon, altitude)
        elif seq == len(mission_items) - 1:  # last waypoint to land
            p = mavutil.mavlink.MAVLink_mission_item_message(
                mav.target_system, mav.target_component, seq, frame, mavutil.mavlink.MAV_CMD_NAV_LAND, current, autocontinue, 0, 0, 0, 0, lat, lon, altitude)
        else:
            p = mavutil.mavlink.MAVLink_mission_item_message(
                mav.target_system, mav.target_component, seq, frame, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, current, autocontinue, 0, 0, 0, 0, lat, lon, altitude)
        wp.add(p)

    mav.wait_heartbeat()

    # Send Waypoint to autopilot
    mav.waypoint_clear_all_send()
    mav.waypoint_count_send(wp.count())

    for i in range(wp.count()):
        msg = mav.recv_match(type=['MISSION_REQUEST'], blocking=True)
        mav.mav.send(wp.wp(msg.seq))
        print('Sending waypoint {0}'.format(msg.seq))

    # OKAY
    msg = mav.recv_match(type=['MISSION_ACK'], blocking=True)


def wait_and_land(mav, mission_items):
    """Wait until mission is completed and land with drone.

    Input:
        mav: pymavlink object with established connection
        mission_items: array of tupples with coordinates (latitude, longitude, altitude) in format 
                        [(lat0,lon0,alt0), (lat1,lon1,alt1),...]
    """
    landed_state = 0

    while True:
        msg = mav.recv_match(
            type=['GLOBAL_POSITION_INT', 'EXTENDED_SYS_STATE', 'HEARTBEAT'], blocking=True, timeout=0.5)
        try:
            if msg.get_type() == 'HEARTBEAT':
                mav.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 192, 0, 4)
            if msg.get_type() == 'EXTENDED_SYS_STATE':
                landed_state = msg.landed_state  # 1 on ground, 2 in air, 3 takeoff, 4 landing
                if landed_state == 1:
                    print("Landed!")
                    break
            if msg.get_type() == 'GLOBAL_POSITION_INT':
                pos_lat = float(msg.lat)*1e-7
                pos_lon = float(msg.lon)*1e-7
                relative_alt = msg.relative_alt

                distance_to_droneport = distance(
                    mission_items[-1][:2], (pos_lat, pos_lon)).km
                if distance_to_droneport < 2e-4:
                    if landed_state == 2:
                        print("Landing on DronePort!")
                        mav.mav.command_long_send(
                            mav.target_system,
                            mav.target_component,
                            mavutil.mavlink.MAV_CMD_NAV_LAND,
                            0,
                            0, 0, 0, 0, 0, 0, 0)
                        msg = mav.recv_match(
                            type=['COMMAND_ACK'], blocking=True)
                        landed_state = 2  # to prevent repeated sending of cmd
            time.sleep(0.1)
            # send heart beat to airframe per 1 sec
            mav.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 192, 0, 4)
        except KeyboardInterrupt:
            break

def reupload_and_resume(mav, infile_name):
    """Reupload saved mission and resume the drone mission from the current waypoint.
    
    Arming of drone is included!

    Input:
        mav: pymavlink object with established connection
        infile_name: string with the name of file
    """
    
    with open(infile_name, 'rb') as infile:
        waypoint_current, waypoint_count, mission_items = pickle.load(infile)

    wp = mavwp.MAVWPLoader()
        
    for waypoint in mission_items:
        waypoint.target_system = mav.target_system
        wp.add(waypoint)
        
    mav.wait_heartbeat()

    # Send Waypoint to autopilot
    mav.waypoint_clear_all_send()
    mav.waypoint_count_send(wp.count())

    for i in range(wp.count()):
        msg = mav.recv_match(type=['MISSION_REQUEST'],blocking=True)
        mav.mav.send(wp.wp(msg.seq))
        print('Sending waypoint {0}'.format(msg.seq))

    # OKAY
    msg = mav.recv_match(type=['MISSION_ACK'],blocking=True)
    time.sleep(0.1)
    # ARM
    mav.mav.command_long_send(mav.target_system, mav.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                            1,
                            0, 0, 0, 0, 0, 0)
    msg = mav.recv_match(type=['COMMAND_ACK'],blocking=True)
    mav.set_mode_auto()

    mav.waypoint_set_current_send(waypoint_current)

    print('Continue to waypoint {0}'.format(waypoint_current))

if __name__ == "__main__":
    """Example with backup a drone mission, send a drone from mission 
    to a DronePort platform location with the intention of replacing the drone battery 
    and finally, return drone to its previous mission.
    
    To run the example, it is REQUIRED to set the coordinates of standard PX4 SITL with following shell commands:
    
    export PX4_HOME_LAT=49.7264592
    export PX4_HOME_LON=13.3508606
    export PX4_HOME_ALT=350.0
    
    Also, it is necessary to start the PX4_SITL f.e. with command
    
    HEADLESS=1 make px4_sitl gazebo
    
    and upload and start the QGroundControl mission (f.e. data/survey_ntis.plan),
    nevertherless, system is able to work with an arbitrary MAVLink mission.
    
    """
    connection_string = '127.0.0.1:14540'   # IP and PORT of drone
    mission_file = 'mission.pkl'  # name of file for saving and reuploading mission
    # array with mission waypoints to droneport: lat, lon coordinates, altitude in meters
    droneport_mission = [(49.7268269, 13.3535106, 50.0)]

    drone = connect(connection_string)
    download_mission(drone, mission_file)

    upload_mission(drone, droneport_mission)
    wait_and_land(drone, droneport_mission)
    reupload_and_resume(drone,mission_file)