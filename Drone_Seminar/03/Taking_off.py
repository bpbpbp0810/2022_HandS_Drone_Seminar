import dronekit
from dronekit import *
import socket
import time

# dronekit-sitl copter --home=37.588478, 127.033843, 0, 0
'''
Always code defensively.
Commands to change a value of settable attributes are not guaranteed to succeed 
(or even to be received) and code should be written with this in mind.
'''

def init_copter(connection_string, airspeed=None):
    vehicle = connect(connection_string, wait_ready=True, heartbeat_timeout=10)
    print("Connected!")

    ### Vehicle is an instance of the Vehicle class
    ### Vehicle state information: attributes
    ### Vehicle settings: parameters
    ### Below are code for getting attributes
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

    # Get Vehicle Home location - will be `None` until first set by autopilot
    # The Home location is set when a vehicle first gets a good location fix from the GPS. 
    # Used for RTL, and the altitude of waypoints will be set relative to this point
    while not vehicle.home_location:
        cmds = vehicle.commands
        cmds.download()
        cmds.wait_ready()
        if not vehicle.home_location:
            print("\tWaiting for home location ...")
    # Now we have a home location.
    print("Home location: %s" % vehicle.home_location) # settable / LocationGlobal object
    # The new location must be within 50 km of the EKF origin or setting the value will silently fail
    # The above code block must be repeated to store a changed home_location value since the previous value is cached

    if airspeed is not None:
        vehicle.airspeed == airspeed
        print("Airspeed set to {0} m/s\n".format(airspeed))

    print("Copter initialization completed!\n\n\n")
    return vehicle

try:
    connection_string = 'tcp:127.0.0.1:5763'
    vehicle = init_copter(connection_string)

    print ("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print ("\tWaiting for vehicle to initialise...")
        time.sleep(1)

    print ("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    # EKF is ready and GPS is locked
    while vehicle.armed == False or vehicle.mode.name != 'GUIDED':
        print("\tWaiting for arming...")
        vehicle.mode = VehicleMode("GUIDED")
        vehicle.armed = True
        time.sleep(1)

    aTargetAltitude = 10
    print ("Taking off!")
    print("Mode: {}".format(vehicle.mode.name))
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print ("\tAltitude: ", vehicle.location.global_relative_frame.alt)
        #Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
            print ("Reached target altitude\n")
            break
        time.sleep(1)


# Bad TCP connection
except socket.error:
    print('No server exists!')

# API Error
except dronekit.APIException:
    print('Timeout!')

# Other error
except Exception as err:
    print(str(err))

print("Vehicle closed")
vehicle.close()