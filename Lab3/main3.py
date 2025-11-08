#!/usr/bin/env pybricks-micropython
'''
Taylor Morris - 730573543
Juhyun Lee - 730793594
'''

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, TouchSensor, UltrasonicSensor, GyroSensor
from pybricks.parameters import Port, Stop, Direction, Button
from pybricks.tools import wait, StopWatch
import math

# ----------------------------
# Configuration
# ----------------------------

ev3 = EV3Brick()     
left_motor = Motor(Port.B)
right_motor = Motor(Port.D)
bump = TouchSensor(Port.S3)   
us = UltrasonicSensor(Port.S4) 
us_motor = Motor(Port.A)
gyro = GyroSensor(Port.S1)

WHEEL_DIAM_MM    = 56.0    
AXLE_TRACK_MM    = 110.0   
FORWARD_SPEED    = 160     
TURN_SPEED       = 120     
WALL_TARGET_MM   = 200.0   
WALL_P           = 1.1    
DIST_CHECK_MS    = 60     

HIT_DETECT_DIST_MM = 300.0 
HIT_RETURN_THRESH_MM = 80.0 
MIN_TRAVEL_BEFORE_DETECT_MM = 200.0  

WALL_FOLLOW_TIMEOUT_MS = 120000
OVERALL_TIMEOUT_MS     = 300000

# ----------------------------
# Helper functions
# ----------------------------

def stop_motors():
    left_motor.stop(Stop.BRAKE)
    right_motor.stop(Stop.BRAKE)

def tank_run(l_spd, r_spd):
    left_motor.run(int(l_spd))
    right_motor.run(int(r_spd))

def scan_with_ultrasonic(num_readings=3, sweep_angle=60):
    """
    Rotates ultrasonic sensor to get better wall readings.
    Returns the minimum distance detected.
    """
    readings = []
    start_angle = us_motor.angle()
    
    # Sweep from -sweep_angle/2 to +sweep_angle/2
    us_motor.run_target(200, start_angle - sweep_angle//2, Stop.HOLD, False)
    wait(50)
    
    for i in range(num_readings):
        readings.append(us.distance())
        if i < num_readings - 1:
            us_motor.run_target(200, start_angle - sweep_angle//2 + (i+1)*(sweep_angle//(num_readings-1)), Stop.HOLD, False)
            wait(50)
    
    # Return to center
    us_motor.run_target(200, start_angle, Stop.HOLD, True)
    wait(50)
    
    # Return minimum distance (closest wall point)
    return min(readings) if readings else us.distance()

def pose_distance(x1, y1, x2, y2):
    return math.hypot(x2 - x1, y2 - y1)

def update_odometry():
    global prev_left_deg, prev_right_deg, pose_x, pose_y, pose_theta
    # Using gyroscope for heading
    pose_theta = math.radians(gyro.angle())
    
    # Useing wheel encoders for distance traveled
    cur_left = left_motor.angle()
    cur_right = right_motor.angle()
    dl_deg = cur_left - prev_left_deg
    dr_deg = cur_right - prev_right_deg
    prev_left_deg = cur_left
    prev_right_deg = cur_right
    
    # Convert to mm
    dl_mm = (dl_deg / 360.0) * math.pi * WHEEL_DIAM_MM
    dr_mm = (dr_deg / 360.0) * math.pi * WHEEL_DIAM_MM
    
    # Average distance traveled
    ds = (dl_mm + dr_mm) / 2.0
    
    # Update position based on current heading
    pose_x += ds * math.cos(pose_theta)
    pose_y += ds * math.sin(pose_theta)

def turn_in_place_right(deg=90):
    # deg: degrees to turn right (positive)
    start = gyro.angle()
    target = start - deg
    tank_run(TURN_SPEED, -TURN_SPEED)
    while gyro.angle() > target:
        update_odometry()
        wait(20)
    stop_motors()

def turn_in_place_left(deg=90):
    if gyro:
        start = gyro.angle()
        target = start + deg
        tank_run(-TURN_SPEED, TURN_SPEED)
        while gyro.angle() < target:
            update_odometry()
            wait(20)
        stop_motors()
    else:
        wheel_deg = deg * (AXLE_TRACK_MM / WHEEL_DIAM_MM)
        left_motor.reset_angle(0)
        right_motor.reset_angle(0)
        tank_run(-TURN_SPEED, TURN_SPEED)
        while True:
            if abs(right_motor.angle()) >= abs(wheel_deg):
                break
            update_odometry()
            wait(10)
        stop_motors()

def accumulate_traveled():
    global last_pose_x, last_pose_y
    dist = pose_distance(pose_x, pose_y, last_pose_x, last_pose_y)
    last_pose_x = pose_x
    last_pose_y = pose_y
    return dist

def angle_normalize(a):
    # normalize angle to [-pi, pi)
    return (a + math.pi) % (2*math.pi) - math.pi

def go_to_goal(goal_x, goal_y, max_run_time_ms=60000):
    timeout = 0
    while True:
        update_odometry()
        dx = goal_x - pose_x
        dy = goal_y - pose_y
        dist = math.hypot(dx, dy)
        if dist < 40:  # within 40 mm -> reached
            stop_motors()
            ev3.speaker.beep()
            break
        # desired heading
        desired_theta = math.atan2(dy, dx)
        heading_error = angle_normalize(desired_theta - pose_theta)
        # if heading error large, rotate in place first
        if abs(heading_error) > math.radians(12):
            # rotate toward target
            sign = 1 if heading_error > 0 else -1
            tank_run(-sign*TURN_SPEED, sign*TURN_SPEED)
        else:
            # go forward with small steering (proportional to heading_error)
            K_turn = 2.0  # steering gain — 조정 필요
            turn = K_turn * heading_error  # rad -> scale
            # convert turn to differential wheel speed offset
            # simply map turn (rad) to speed difference
            base = 140
            offset = (turn / 0.5) * 80 
            lsp = base + offset
            rsp = base - offset
            lsp = max(min(lsp, 400), -300)
            rsp = max(min(rsp, 400), -300)
            tank_run(lsp, rsp)
        wait(60)
        timeout += 60
        if timeout > max_run_time_ms:
            stop_motors()
            ev3.speaker.beep()
            break

# ---------------------------
# Odometry state
# ---------------------------

# Store current position
pose_x = 0.0
pose_y = 0.0
pose_theta = 0.0

# Store previous motor encoder angles
prev_left_deg = left_motor.angle()
prev_right_deg = right_motor.angle()

# Store last x-y pose
last_pose_x = 0.0
last_pose_y = 0.0

# ----------------------------
# Main
# ----------------------------

while Button.CENTER not in ev3.buttons():
    wait(50)

# Reset gyro to 0 at start
gyro.reset_angle(0)
wait(100)

# === Step 1: Move forward until obstacle detected, save hit point ===
tank_run(FORWARD_SPEED, FORWARD_SPEED)

# reset prev angles for odometry accumulation from start
prev_left_deg = left_motor.angle()
prev_right_deg = right_motor.angle()

elapsed = 0
while True:
    d = us.distance()  # mm
    update_odometry()

    if d <= HIT_DETECT_DIST_MM:
        stop_motors()
        ev3.speaker.beep()
        wait(150)
        break
    wait(DIST_CHECK_MS)
    elapsed += DIST_CHECK_MS
    if elapsed > OVERALL_TIMEOUT_MS:
        stop_motors()

# record hit pose (x,y,theta) 
hit_pose = (pose_x, pose_y, pose_theta)

# === Step 2: Turn right 90 degrees ===

turn_in_place_right(90)
wait(200)

# === Step 3: Wall Following around obstacle ===
base_speed = 140
elapsed = 0
total_traveled_since_hit = 0.0

while True:
    stop_motors()
    d = scan_with_ultrasonic(num_readings=3, sweep_angle=60)
    
    update_odometry()
    traveled = accumulate_traveled()
    total_traveled_since_hit += traveled

    # P control for wall-follow (right side)
    # error = measured - target: positive -> too far -> need to steer toward wall (turn right)
    error = d - WALL_TARGET_MM
    correction = WALL_P * error / 10.0  # scale down
    # For right-side following, steer so that if d > target (too far), we turn right (increase left wheel speed)
    left_speed = base_speed + correction
    right_speed = base_speed - correction
    # clamp
    left_speed = max(min(left_speed, 400), -300)
    right_speed = max(min(right_speed, 400), -300)
    tank_run(left_speed, right_speed)

    wait(DIST_CHECK_MS)
    elapsed += DIST_CHECK_MS

    # If we lose the wall, back up slightly and re-scan
    if d > 800:
        stop_motors()
        tank_run(-100, -100)
        wait(200)
        stop_motors()
        d = scan_with_ultrasonic(num_readings=2, sweep_angle=45)

    # check if back near hit_pose
    dist_to_hit = pose_distance(pose_x, pose_y, hit_pose[0], hit_pose[1])
    # require we have traveled at least MIN_TRAVEL_BEFORE_DETECT_MM to avoid immediate detection
    if dist_to_hit <= HIT_RETURN_THRESH_MM and total_traveled_since_hit >= MIN_TRAVEL_BEFORE_DETECT_MM:        
        stop_motors()
        ev3.speaker.beep()
        wait(200)
        break

    if elapsed > WALL_FOLLOW_TIMEOUT_MS:
        stop_motors()
        ev3.speaker.beep()
        break

# === Step 4: Going back to starting point ===

turn_in_place_left(90)
wait(200)

# go to start (0,0)
go_to_goal(0.0, 0.0, max_run_time_ms=90000)
stop_motors()
ev3.speaker.beep()