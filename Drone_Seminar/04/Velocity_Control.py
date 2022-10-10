import dronekit
from dronekit import *
import socket
import time
from haversine import haversine
# import math, sys

# dronekit-sitl copter --home=37.588478, 127.033843, 0, 0
'''
Always code defensively.
Commands to change a value of settable attributes are not guaranteed to succeed 
(or even to be received) and code should be written with this in mind.
'''

point_dict = {'정문': LocationGlobalRelative(37.588083, 127.034306, 10), 
              '경영본관': LocationGlobalRelative(37.590381, 127.035096, 10),
              '본관': LocationGlobalRelative(37.589531, 127.032297, 10),
              '민주광장': LocationGlobalRelative(37.587246, 127.031761, 10)}


def init_copter(connection_string, airspeed=None):
    vehicle = connect(connection_string, wait_ready=True, heartbeat_timeout=10)
    print("Connected!")

    ### Vehicle is an instance of the Vehicle class.
    ### vehicle state information: attributes
    ### vehicle settings: parameters
    ### Below are code lines for getting attributes.
    print("Reading attributes...")
    print("Autopilot Firmware version: %s" % vehicle.version)
    print("Autopilot capabilities (supports ftp): %s" % vehicle.capabilities.ftp)
    print("Global Location: %s" % vehicle.location.global_frame)
    print("Global Location (relative altitude): %s" % vehicle.location.global_relative_frame)
    print("Local Location: %s" % vehicle.location.local_frame)
    print("Attitude: %s" % vehicle.attitude)
    print("Velocity: %s" % vehicle.velocity)
    print("GPS: %s" % vehicle.gps_0)
    print("Groundspeed: %s" % vehicle.groundspeed)  # settable
    print("Airspeed: %s" % vehicle.airspeed)    # settable
    print("Gimbal status: %s" % vehicle.gimbal) # settable indirectly by other methods (angle, gps info of roi)
    print("Battery: %s" % vehicle.battery)
    print("EKF OK?: %s" % vehicle.ekf_ok)
    print("Last Heartbeat: %s" % vehicle.last_heartbeat)
    print("Rangefinder: %s" % vehicle.rangefinder)
    print("Rangefinder distance: %s" % vehicle.rangefinder.distance)
    print("Rangefinder voltage: %s" % vehicle.rangefinder.voltage)
    print("Heading: %s" % vehicle.heading)
    print("Is Armable?: %s" % vehicle.is_armable)
    print("System status: %s" % vehicle.system_status.state) # Apps could monitor Vehicle.system_status for CRITICAL or EMERGENCY in order to implement specific emergency handling.
    print("Mode: %s" % vehicle.mode.name)    # settable
    print("Armed: %s" % vehicle.armed)    # settable

    # Get Vehicle Home location - will be `None` until first set by autopilot.
    # The Home location is set when a vehicle first gets a good location fix from the GPS. 
    # Used for RTL, and the altitude of waypoints will be set relative to this point.
    while not vehicle.home_location:
        cmds = vehicle.commands
        cmds.download()
        cmds.wait_ready()
        if not vehicle.home_location:
            print("\tWaiting for home location ...")
    # Now we have a home location.
    print("Home location: %s" % vehicle.home_location) # settable / LocationGlobal object
    # The new location must be within 50 km of the EKF origin or setting the value will silently fail.
    # The above code block must be repeated to store a changed home_location value since the previous value is cached.

    if airspeed is not None:
        vehicle.airspeed == airspeed
        print("Airspeed set to {0} m/s\n".format(airspeed))

    print("Copter initialization completed!\n\n")
    return vehicle


def take_off(vehicle, aTargetAltitude):
    print ("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready.
    while not vehicle.is_armable:
        print ("\tWaiting for vehicle to initialise...")
        time.sleep(1)

    print ("Arming motors")
    # Copter should arm in GUIDED mode.
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off.
    # EKF is ready and GPS is locked.
    while vehicle.armed == False or vehicle.mode.name != 'GUIDED':
        print("\tWaiting for arming...")
        vehicle.mode = VehicleMode("GUIDED")
        vehicle.armed = True
        time.sleep(1)

    print ("Taking off!")
    print("Mode: {}".format(vehicle.mode.name))
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude.

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print ("\tAltitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
            print ("Reached target altitude\n\n")
            break
        time.sleep(1)


def goto_position(vehicle, targetLocation, groundspeed = 1):    
    ########## How to use haversine ##########
    # currentLocation = vehicle.location.global_relative_frame
    # a = (currentLocation.lat, currentLocation.lon)
    # b = (targetLocation.lat, targetLocation.lon)
    # targetDistance = haversine(a, b, unit = 'm')   # distance between the current position and target position

    for key, value in point_dict.items():
        if targetLocation == value:
            target = key
            break
        else:
            target = 'target not in point_dict'

    print("Heading to the target location")
    print("Target:", target)
    print("Target location: lat={}, lon={}".format(targetLocation.lat, targetLocation.lon))
    
    print("Groundspeed set to {} m/s".format(groundspeed)) 
    vehicle.simple_goto(targetLocation, groundspeed=groundspeed)

    while vehicle.mode.name=="GUIDED": # Halt if no longer in guided mode.
        b = (targetLocation.lat, targetLocation.lon)
        c = (vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon)
        remainingDistance = haversine(c, b, unit = 'm')
        print("\tDistance to target: {:.2f}m".format(remainingDistance)) 

        if remainingDistance <= 1: # Just below target, in case of undershoot.
            print("Reached target\n\n")
            break 

        time.sleep(1)


def send_ned_velocity(vehicle, velocity_x, velocity_y, velocity_z, duration):
    # Set up velocity mappings
    # velocity_x > 0 => fly North
    # velocity_x < 0 => fly South
    # velocity_y > 0 => fly East
    # velocity_y < 0 => fly West
    # velocity_z < 0 => ascend
    # velocity_z > 0 => descend
    
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # frame
        # mavutil.mavlink.MAV_FRAME_BODY_FRD,   # for Copter versions released after 2019-08
        # https://mavlink.io/en/messages/common.html#MAV_FRAME_BODY_OFFSET_NED 
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    msg_immediate_stop = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # frame
        # mavutil.mavlink.MAV_FRAME_BODY_FRD,   # for Copter versions released after 2019-08
        # https://mavlink.io/en/messages/common.html#MAV_FRAME_BODY_OFFSET_NED 
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        0, 0, 0, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # The message is re-sent every second during the specified duration. 
    # From Copter 3.3 the vehicle will stop moving if a new message is not received in approximately 3 seconds.
    for x in range(0, duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)

    # Send a stop messeage twice in case of not being received.
    vehicle.send_mavlink(msg_immediate_stop)
    vehicle.send_mavlink(msg_immediate_stop)


def rtl(vehicle):
    print ("Returning home")
    vehicle.parameters['RTL_ALT'] = 0
    time.sleep(1)

    vehicle.mode = VehicleMode("RTL")
    while vehicle.mode.name != 'RTL':
        vehicle.mode = VehicleMode("RTL")
        time.sleep(1)

    print("Mode: {}".format(vehicle.mode.name))

    while vehicle.mode.name == "RTL":
        h = (vehicle.home_location.lat, vehicle.home_location.lon)
        c = (vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon)
        remainingDistance = haversine(c, h, unit = 'm')
        print("\tDistance to target: {:.2f}m".format(remainingDistance)) 

        if remainingDistance <= 0.5: # Just below target, in case of undershoot.
            print("Arrived home")
            break 

        time.sleep(1)

    while vehicle.location.global_relative_frame.alt - vehicle.home_location.alt > 0.5: 
        print("\tLanding...")
        time.sleep(1)

    print("Vehicle on the ground")
    print("Close vehicle object\n\n")
    vehicle.close()


try:
    connection_string = 'tcp:127.0.0.1:5763'
    copter = init_copter(connection_string)
    time.sleep(2)

    target_alt = 10
    take_off(copter, target_alt)
    time.sleep(2)

    goto_position(copter, point_dict['본관'], groundspeed=8)
    time.sleep(2)

    send_ned_velocity(copter, 8, 0, 0, 5)
    time.sleep(2)    

    send_ned_velocity(copter, 0, 8, 0, 5)
    time.sleep(2) 

    send_ned_velocity(copter, -8, 0, 0, 5)
    time.sleep(2)    

    send_ned_velocity(copter, 0, -8, 0, 5)
    time.sleep(2)    

# Bad TCP connection
except socket.error:
    print('No server exists!')

# API Error
except dronekit.APIException:
    print('Timeout!')

# Other error
except Exception as err:
    print("Error occurred")
    print(str(err))

rtl(copter)

