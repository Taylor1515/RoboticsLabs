
def follow_wall_until_return(hit_pose):
    """
    Follow the right wall using a simple P controller until we return near the saved hit pose.
    - hit_pose: tuple (hx, hy, htheta) recorded right after backing up from the front wall.
    - Uses: odometry (pose_x, pose_y, pose_theta) + ultrasonic.
    - Stop when: close to hit_pose AND traveled enough AND ultrasonic says near the wall again.
    """

    # ---------- Tunable parameters ----------
    base_speed = 150          # forward speed (deg/s)
    kp = 1.0                  # P gain for wall distance
    setpoint_mm = 200         # desired distance to wall (right side), ~20 cm
    max_delta = 40            # max steering offset (deg/s)
    max_speed = 300           # speed clamp (deg/s)

    hit_return_thresh_mm = 100        # distance to hit pose to accept "we're back" (80~120)
    min_travel_before_detect_mm = 250 # avoid instant false positive
    near_wall_threshold_mm = 300      # ultrasonic must say we're near wall/front face
    us_window_size = 5                # smoothing window for ultrasonic
    loop_dt_ms = 40                   # loop period
    timeout_ms = 120000               # safety timeout

    # ---------- Prep ----------
    hx, hy, _ = hit_pose
    us_window = []
    elapsed = 0

    reset_wheels()  # start counting traveled distance from here

    # ---------- Control loop ----------
    while True:
        # (1) Read ultrasonic and smooth it (median)
        d = us.distance()  # mm (can be noisy)
        us_window.append(d)
        if len(us_window) > us_window_size:
            us_window.pop(0)
        d_sorted = sorted(us_window)
        d_med = d_sorted[len(d_sorted)//2]  # median smoothing

        # (2) P-control for right wall-follow
        #     error > 0 => too far => steer right (speed up left, slow down right)
        error = d_med - setpoint_mm
        delta = kp * error
        # clamp steering
        if delta > max_delta:   delta = max_delta
        if delta < -max_delta:  delta = -max_delta

        l_cmd = base_speed + delta
        r_cmd = base_speed - delta

        # speed clamp
        l_cmd = max(min(l_cmd,  max_speed), -max_speed)
        r_cmd = max(min(r_cmd,  max_speed), -max_speed)

        drive(l_cmd, r_cmd)

        # (3) Check return condition
        dist_to_hit = ((pose_x - hx)**2 + (pose_y - hy)**2) ** 0.5
        traveled = traveled_mm_since_reset()

        # Must be: close to hit_pose + traveled enough + near the wall/front again
        if (dist_to_hit <= hit_return_thresh_mm and
            traveled >= min_travel_before_detect_mm and
            d_med <= near_wall_threshold_mm):
            stop_motors()
            break

        # (4) Timeout as safety
        wait(WAITING_TIME)
        elapsed += WAITING_TIME
        if elapsed >= timeout_ms:
            stop_motors()
            break


# --- Simple right wall-follow until we return near the saved hit pose ---
# Assumes these exist elsewhere:
#  - Globals: pose_x, pose_y, pose_theta  (odometry kept up to date by your main loop or per-iteration)
#  - Ultrasonic sensor: us.distance() -> mm
#  - Helpers: drive(l_cmd, r_cmd), stop_motors(), reset_wheels(), traveled_mm_since_reset()
#             turn_in_place(deg), move_straight_distance(dist_mm, speed)

follow_wall_until_return(hit_pose)