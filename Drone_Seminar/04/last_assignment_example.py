# print("Start simulator (SITL)")
# from time import sleep
# import dronekit_sitl
# sitl = dronekit_sitl.start_default()
# connection_string = sitl.connection_string()
#################################################
# simulation code starts

from dronekit import connect,VehicleMode, LocationGlobalRelative
# LocationGlobalRelative(lat, lon, alt=None) => A global location object, with altitude relative to home location altitude.
# LocationGlobal(lat, lon, alt=None) => A global location object, with altitude relative to mean sea level
from pymavlink import mavutil # Needed for command message definitions
import dronekit
import socket
import time
import threading
from haversine import haversine
import cv2
import numpy as np
import keyboard
from queue import Queue
from easydict import EasyDict

q_pressed = 0
lock = 0

status = EasyDict({
    'patrol': 'patrol',
    'aim': 'aim',
    'drop': 'drop',
    'rtl': 'rtl',
    'wait': 'wait',
    'gotofire': 'gotofire'})
v1_status = status.patrol   # patrol로 시작
v2_status = status.wait

# Multiple vehicle 설정: connect/disconnect 버튼 우클릭 -> 연결 옵션 -> 맞는 포트 번호 및 보드 레이트 설정하고 확인 누르기
# dronekit-sitl copter -I0 --home=37.59154667413565, 127.02494844832526,0,0
# connection_string_1 = 'tcp:127.0.0.1:5763'    # 5760에 연결
# dronekit-sitl copter -I1 --home=37.59154667413565, 127.02494844832526,0,0
# connection_string_2 = 'tcp:127.0.0.1:5773'  # 5770에 연결
# dronekit-sitl cotper -I2 --home=좌표
# connection_string_3 = 'tcp:127.0.0.1:5773'  # 5780에 연결
# ...

point1 = LocationGlobalRelative(37.59154667413565, 127.02494844832526, 10) # 녹지운동장
point2 = LocationGlobalRelative(37.59272000442947, 127.0248748575227, 10)  # 화정체육관
# point3 = LocationGlobalRelative(37.590677960237706, 127.0266865843291, 10) # 생명과학관  
# point4 = LocationGlobalRelative(37.588083, 127.034306, 10)  # 정문
# point5 = LocationGlobalRelative(37.590381, 127.035096, 10)  # 경영본관
# point6 = LocationGlobalRelative(37.589531, 127.032297, 10)  # 본관
# point7 = LocationGlobalRelative(37.587246, 127.031761, 10)  # 민주광장

points = [point2, point1]
point_idx = 0

# Always code defensivelqy
# Commands to change a value of settable attributes are not guaranteed to succeed (or even to be received) 
# and code should be written with this in mind.


def vehicle_init(connection_string, vehicle_num, airspeed = None):
    print("Connecting to vehicle on: %s" % connection_string)
    vehicle = connect(connection_string, wait_ready=True)
    print('Vehicle' + str(vehicle_num) + 'connected!')

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
    # The new location must be within 50 km of the EKF origin or setting the value will silently fail
    # The above code block must be repeated to store a changed home_location value since the previous value is cached

    if airspeed is not None:
        vehicle.airspeed == airspeed
        print("Airspeed set to {0} m/s\n".format(airspeed))

    return vehicle


def arm_and_takeoff(vehicle, aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    # print ("Basic pre-arm checks")
    # # Don't try to arm until autopilot is ready
    # while not vehicle.is_armable:
    #     print ("\tWaiting for vehicle to initialise...")
    #     time.sleep(1)

    print ("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode    = VehicleMode("GUIDED")
    vehicle.armed   = True

    # Confirm vehicle armed before attempting to take off
    # EKF is ready and GPS is locked
    while vehicle.armed == False or vehicle.mode.name != 'GUIDED':
        print("\tWaiting for arming...")
        time.sleep(1)

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


def goto(vehicle, targetLocation, v1_Rx, groundspeed = None):
    """
    Moves the vehicle to a position dNorth metres North and dEast metres East of the current position.

    The method takes a function pointer argument with a single `dronekit.lib.LocationGlobal` parameter for 
    the target position. This allows it to be called with different position-setting commands. 
    By default it uses the standard method: dronekit.lib.Vehicle.simple_goto().

    The method reports the distance to target every two seconds.
    """
    
    # currentLocation = vehicle.location.global_relative_frame    # starting point
    # a = (currentLocation.lat, currentLocation.lon)  # 출발 위치 위도 경도 (출발점)
    # b = (targetLocation.lat, targetLocation.lon)    # 타겟 위치 위도 경도 (도착점)
    # targetDistance = haversine(a, b, unit = 'm')   # 출발점과 도착점 사이의 거리
    
    global v1_status
    global q_pressed

    print("Heading to the target location")
    print("Target location:", targetLocation)
    
    if groundspeed is not None:
        vehicle.simple_goto(targetLocation, groundspeed=groundspeed)
        print("Groundspeed set to {0} m/s".format(groundspeed)) 
    else:
        vehicle.simple_goto(targetLocation)

    while vehicle.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
        #print "DEBUG: mode: %s" % vehicle.mode.name
        b = (targetLocation.lat, targetLocation.lon)    # 도착점 위도 경도
        c = (vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon)    # 비행 중 현재 위치 위도 경도
        remainingDistance = haversine(c, b, unit = 'm')   # 도착점까지 남은 거리
        print("\tDistance to target: ", remainingDistance) 

        if keyboard.is_pressed('q'):
            print("!!!!!!!!!! q is pressed !!!!!!!!!!")
            q_pressed = 1
            return 
        
        if v1_Rx.empty() is False:  # # v1_Rx가 비어있지 않을 때만 dequeue
            v1_status = v1_Rx.get()
            return

        if remainingDistance <= 1: # 남은 거리가 1미터 이하면 도착한거로 판단
            #Just below target, in case of undershoot.
            print("Reached target")
            break 

        time.sleep(1)


def goto_fire(vehicle, v1_Tx, groundspeed = None):
    """
    Moves the vehicle to a position dNorth metres North and dEast metres East of the current position.

    The method takes a function pointer argument with a single `dronekit.lib.LocationGlobal` parameter for 
    the target position. This allows it to be called with different position-setting commands. 
    By default it uses the standard method: dronekit.lib.Vehicle.simple_goto().

    The method reports the distance to target every two seconds.
    """
    
    # currentLocation = vehicle.location.global_relative_frame    # starting point
    # a = (currentLocation.lat, currentLocation.lon)  # 출발 위치 위도 경도 (출발점)
    # b = (targetLocation.lat, targetLocation.lon)    # 타겟 위치 위도 경도 (도착점)
    # targetDistance = haversine(a, b, unit = 'm')   # 출발점과 도착점 사이의 거리

    while v1_Tx.empty() is True:
        time.sleep(0.1)
    
    targetLocation = v1_Tx.get()

    print("Fire location is received.")
    print("Heading to the fire location")
    print("Fire location:", targetLocation)
    
    if groundspeed is not None:
        vehicle.simple_goto(targetLocation, groundspeed=groundspeed)
        print("Groundspeed set to {0} m/s".format(groundspeed)) 
    else:
        vehicle.simple_goto(targetLocation)

    while vehicle.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
        #print "DEBUG: mode: %s" % vehicle.mode.name
        b = (targetLocation.lat, targetLocation.lon)    # 도착점 위도 경도
        c = (vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon)    # 비행 중 현재 위치 위도 경도
        remainingDistance = haversine(c, b, unit = 'm')   # 도착점까지 남은 거리
        print("\tDistance to target: ", remainingDistance) 

        if remainingDistance <= 1: # 남은 거리가 1미터 이하면 도착한거로 판단
            #Just below target, in case of undershoot.
            print("Reached Fire")
            break 

        time.sleep(1)


def patrol(vehicle, points, v1_Rx):
    global q_pressed
    global point_idx
    global v1_status

    while True:
        
        print("Heading to the next point\n")
        goto(vehicle, points[point_idx], v1_Rx, 5)

        if q_pressed == 1:
            print("!!!!!!!!!! EMERGENCY !!!!!!!!!!")
            print("Entering RTL mode")
            v1_status = status.rtl
            return

        if v1_status == status.aim:
            return 

        if point_idx == len(points) - 1:
            point_idx = 0
        else:
            point_idx = point_idx + 1

        time.sleep(0.5)


def send_ned_velocity(vehicle, velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    The function send_ned_velocity() below generates a SET_POSITION_TARGET_LOCAL_NED MAVLink message 
    which is used to directly specify the speed components of the vehicle in the MAV_FRAME_LOCAL_NED frame 
    (relative to home location). The message is re-sent every second for the specified duration.
    """
    
    global q_pressed

    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # The message is re-sent every second for the specified duration. 
    # From Copter 3.3 the vehicle will stop moving if a new message is not received in approximately 3 seconds.
    for _ in range(0,int(2 * duration)):
        if keyboard.is_pressed('q'):
            print("!!!!!!!!!! q is pressed !!!!!!!!!!")
            q_pressed = 1
            return 
        vehicle.send_mavlink(msg)
        time.sleep(0.5)


def aim(vehicle, v1_Rx, v1_Tx):
    global v1_status
    global v2_status
    global lock

    tik_tok = 0
    while True: 
        

        if q_pressed == 1:
            print("!!!!!!!!!! EMERGENCY !!!!!!!!!!")
            print("Entering RTL mode")
            v1_status = status.rtl
            return

        # aim 모드일 때는 무조건 좌표값을 받아와야함   
        while v1_Rx.empty() is True: 
            time.sleep(0.1)

        cvx, cvy = v1_Rx.get()  # v1_Rx에 좌표가 enqueue 될 때까지 기다리다가 받아오기
        print("cvx: ", cvx, ", cvy: ", cvy)

        if cvx is None or cvy is None:  # 불을 놓치거나(None, None) 잘못된 좌표값(둘 중 하나가 None)이 들어왔을 때
            tik_tok = 0
            print("Received wrong info\n")
            start_time = time.time()
            while True:
                if time.time() - start_time >= 5.0: # 5초동안 제대로 된 값이 안들어오면 다시 patrol
                    print("Back to patrol")
                    v1_status = status.patrol
                    return

                while v1_Rx.empty() is True: # 좌표 받기 계속 시도
                    time.sleep(0.1)
                cvx, cvy = v1_Rx.get()
                print("cvx: ", cvx, ", cvy: ", cvy)
                if cvx is not None and cvy is not None:  # 제대로 받아 왔으면 루프 탈출
                    print("Received correct info\n")
                    break
                
        else:   # 좌표가 제대로 들어 왔을 때 자동 조준
            if cvy > 50:
                tik_tok = 0
                send_ned_velocity(vehicle,5,0,0,0.5) 
            elif cvy < -50:
                tik_tok = 0
                send_ned_velocity(vehicle,-5,0,0,0.5)
            else:
                if cvx > 50:
                    tik_tok = 0
                    send_ned_velocity(vehicle,0,-5,0,0.5)
                elif cvx < -50:
                    tik_tok = 0
                    send_ned_velocity(vehicle,0,5,0,0.5)
                else:
                    tik_tok = tik_tok + 1
                    print("Target in range")
                    print("Holding breath", tik_tok)
                    time.sleep(1)
                    
                    if tik_tok == 5:    # 5초 동안 조준 유지 하면 소화탄 떨구기로 진행
                        lock = 1
                        _ = v1_Rx.get()
                        v1_status = status.drop
                        v2_status = status.gotofire
                        firelocation = vehicle.location.global_relative_frame
                        while v1_Tx.empty() is False:
                            time.sleep(0.1)
                        v1_Tx.put(firelocation)  # vehicle2 한테 보낼 불의 좌표
                        print("Fire location is sent to vehicle 2")
                        return
        

def drop():
    print("Drop a fire extinguisher bomb")
    # 라즈베리파이 GPIO 핀 조작 코드
    time.sleep(3)
    print("Dropped")
    return


def rtl(vehicle):
    print ("Returning to launch")
    vehicle.parameters['RTL_ALT'] = 0
    time.sleep(1)
    vehicle.mode = VehicleMode("RTL")
    time.sleep(1)
    print("Mode: {}".format(vehicle.mode.name))

    while vehicle.mode.name == "RTL":
        h = (vehicle.home_location.lat, vehicle.home_location.lon)
        c = (vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon)    # 비행 중 현재 위치 위도 경도
        remainingDistance = haversine(c, h, unit = 'm')   # 도착점까지 남은 거리
        #print("\tDistance to home: ", remainingDistance)
        if remainingDistance <= 1: # 남은 거리가 1미터 이하면 도착한거로 판단
                #Just below target, in case of undershoot.
                print("Reached home")
                break 
        time.sleep(1)

    while vehicle.location.global_relative_frame.alt - vehicle.home_location.alt > 1: 
        #print("\tLanding...")
        time.sleep(1)

    print("Vehicle landed on home location")
    print("Close vehicle object")
    vehicle.close()
    return


def opencv(v1_Rx):
    global v1_status
    global lock
    
    cap = cv2.VideoCapture(0)
    width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

    while True:

        _, frame = cap.read()
        cvx = None
        cvy = None

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        l_b = np.array([0, 150, 150])
        u_b = np.array([25, 360, 360])

        mask = cv2.inRange(hsv, l_b, u_b)
        blur = cv2.GaussianBlur(mask, (5, 5), 0)
        _, thresh = cv2.threshold(blur, 10, 255, cv2.THRESH_BINARY)
        dilated = cv2.dilate(thresh, None, iterations=3)
        contours, _ = cv2.findContours(dilated, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        if len(contours) != 0: 
            largest = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest) > 1000:
                (x, y, w, h) = cv2.boundingRect(largest)        
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3)
                cvx = int((x+w/2)-320)
                cvy = int((y+h/2)-240)
                cv2.putText(frame, "Area : " + str(w*h), (x-10, y-25), cv2.FONT_ITALIC, 0.5, (255, 20, 20), 2)
                cv2.putText(frame, "Distance_from_center : (" + str(cvx) + "," +str(cvy) + ")", (x-10, y-10), cv2.FONT_ITALIC, 0.5, (255, 20, 20), 2)

        cv2.rectangle(frame, (int(width / 2) - 50, int(height / 2) - 50), (int(width / 2) + 50, int(height / 2) + 50), (255, 0, 0), 3)
        cv2.line(frame, (int(width / 2) - 10, int(height / 2)), (int(width / 2) + 10, int(height / 2)), (0, 0, 255), 3)
        cv2.line(frame, (int(width / 2), int(height / 2) - 10), (int(width / 2), int(height / 2) + 10), (0, 0, 255), 3)
        cv2.putText(frame, 'v1 status: '+v1_status, (5, 35), cv2.FONT_ITALIC, 1, (0, 0, 0), 2)
        cv2.putText(frame, 'v2_status: '+v2_status, (5, 80), cv2.FONT_ITALIC, 1, (0, 0, 0), 2)
        cv2.imshow("final", frame)
        if v1_Rx.empty() is True and lock == 0:
            if (cvx is not None and cvy is not None) and v1_status == status.patrol:
                v1_Rx.put('aim')
            elif v1_status == status.aim:
                v1_Rx.put((cvx, cvy))
                

        if cv2.waitKey(100) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


def vehicle1_flight_thread(v1_Rx):
    global v1_status
    global points
    global lock

    connection_string_1 = 'tcp:127.0.0.1:5763'    # 5760에 연결
    vehicle1 = vehicle_init(connection_string_1, 1)
    arm_and_takeoff(vehicle1, 10)

    while True:
        if v1_status == status.patrol:
            patrol(vehicle1, points, v1_Rx)
        elif v1_status == status.aim:
            aim(vehicle1, v1_Rx, v1_Tx)
        elif v1_status == status.drop:
            drop()
            lock = 0
            v1_status = status.rtl
        elif v1_status == status.rtl:
            rtl(vehicle1)
            return


def vehicle2_flight_thread(v1_Tx):
    global v2_status

    connection_string_2 = 'tcp:127.0.0.1:5773'  # 5770에 연결
    vehicle2 = vehicle_init(connection_string_2, 2)
    
    while True:
        if v2_status == status.wait:
            time.sleep(1)
        elif v2_status == status.gotofire:
            arm_and_takeoff(vehicle2, 10)
            goto_fire(vehicle2, v1_Tx, 5)
            v2_status = status.drop
        elif v2_status == status.drop:
            drop()
            v2_status = status.rtl
        elif v2_status == status.rtl:
            rtl(vehicle2)
            return


try:
    v1_Rx = Queue()
    v1_Tx = Queue()

    t1 = threading.Thread(target=vehicle1_flight_thread, args=(v1_Rx, ))
    t2 = threading.Thread(target=vehicle2_flight_thread, args=(v1_Tx, ))
    oc1 = threading.Thread(target=opencv, args=(v1_Rx, ))

    t1.start()
    t2.start()
    oc1.start()

    t1.join()
    t2.join()
    oc1.join()
            
# Bad TCP connection
except socket.error:
    print('No server exists!')

# Bad TTY connection
except OSError:
    print('No serial exists!')

# API Error
except dronekit.APIException:
    print('Timeout!')

# Other error
except Exception as err:
    print(str(err))


# simulation code ends
######################################################
# sitl.stop()
# print("Completed")