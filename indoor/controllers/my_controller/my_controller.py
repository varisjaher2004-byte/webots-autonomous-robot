from controller import Supervisor
import math

# ===================== ADD (CV IMPORTS) =====================
import cv2
import numpy as np
# ===========================================================

robot = Supervisor()
TIME_STEP = int(robot.getBasicTimeStep())

# ----------------------------------------------------
#  SENSORS
# ----------------------------------------------------
gs = []
for i in range(3):
    dev = robot.getDevice(f"gs{i}")
    dev.enable(TIME_STEP)
    gs.append(dev)

gps = robot.getDevice("gps")
gps.enable(TIME_STEP)

ps = []
for i in range(8):
    dev = robot.getDevice(f"ps{i}")
    dev.enable(TIME_STEP)
    ps.append(dev)

imu = robot.getDevice("inertial unit")
imu.enable(TIME_STEP)

try:
    ls = robot.getDevice("light sensor")
    ls.enable(TIME_STEP)
except Exception:
    ls = None

# ===================== ADD (CAMERA ENABLE) =====================
camera = robot.getDevice("camera")
camera.enable(TIME_STEP)
CAM_W = camera.getWidth()
CAM_H = camera.getHeight()
# ===============================================================

# ----------------------------------------------------
#  MOTORS
# ----------------------------------------------------
left_motor  = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")

left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# ----------------------------------------------------
#  LIGHT NODES
# ----------------------------------------------------
bedroom_node = robot.getFromDef("BEDROOM_")
lounge_node  = robot.getFromDef("LOUNDGE_")
kitchen_node = robot.getFromDef("KITCHEN_")

bedroom_on = bedroom_node.getField("on") if bedroom_node else None
lounge_on  = lounge_node.getField("on")  if lounge_node  else None
kitchen_on = kitchen_node.getField("on") if kitchen_node else None

# ----------------------------------------------------
#  ROOM DETECTION (GS COLOR)
# ----------------------------------------------------
def detect_room(gs_vals):
    avg = sum(gs_vals) / 3.0
    if avg < 330:
        return "KITCHEN"
    elif avg < 680:
        return "BEDROOM"
    else:
        return "LOUNGE"

# ----------------------------------------------------
#  WALL CHECK
# ----------------------------------------------------
def near_wall(ps_vals):
    return max(ps_vals) > 120.0

# ----------------------------------------------------
#  BEDROOM WALL FREEZE FIX
# ----------------------------------------------------
def freeze_if_bedroom_wall(ps_vals, gps_vals):
    x = gps_vals[0]
    z = gps_vals[1]
    return (
        -1.05 < x < -0.85 and
        -0.10 < z < 0.30 and
        max(ps_vals) > 70
    )

# ----------------------------------------------------
#  GPS ROOM OVERRIDE
# ----------------------------------------------------
def gps_room_override(x, z):
    if -1.10 < x < -0.72 and -0.08 < z < 0.40:
        return "BEDROOM"
    if -1.10 < x < -0.55 and -1.10 < z < -0.35:
        return "KITCHEN"
    if 0.20 < x < 0.90 and -0.25 < z < 0.40:
        return "LOUNGE"
    return None

# ----------------------------------------------------
#  STABLE ROOM LOGIC
# ----------------------------------------------------
stable_room = None
room_change_counter = 0
STABLE_LIMIT = 15

def get_stable_room(raw_room, override=None):
    global stable_room, room_change_counter

    if override is not None:
        stable_room = override
        room_change_counter = 0
        return stable_room

    if raw_room != stable_room:
        room_change_counter += 1
        if room_change_counter >= STABLE_LIMIT:
            stable_room = raw_room
            room_change_counter = 0
    else:
        room_change_counter = 0

    return stable_room

# ----------------------------------------------------
#  LIGHT CONTROL (BLINK FIXED)
# ----------------------------------------------------
prev_light_room = None
light_candidate = None
light_counter = 0
LIGHT_STABLE_STEPS = 12

def update_lights(stable_room):
    global prev_light_room, light_candidate, light_counter

    if stable_room is None:
        return

    if prev_light_room is None:
        if bedroom_on: bedroom_on.setSFBool(False)
        if lounge_on:  lounge_on.setSFBool(False)
        if kitchen_on: kitchen_on.setSFBool(False)

        if stable_room == "BEDROOM": bedroom_on.setSFBool(True)
        elif stable_room == "LOUNGE": lounge_on.setSFBool(True)
        elif stable_room == "KITCHEN": kitchen_on.setSFBool(True)

        prev_light_room = stable_room
        return

    if stable_room == prev_light_room:
        light_candidate = None
        light_counter = 0
        return

    if light_candidate != stable_room:
        light_candidate = stable_room
        light_counter = 1
        return
    else:
        light_counter += 1

    if light_counter < LIGHT_STABLE_STEPS:
        return

    if bedroom_on: bedroom_on.setSFBool(False)
    if lounge_on:  lounge_on.setSFBool(False)
    if kitchen_on: kitchen_on.setSFBool(False)

    if stable_room == "BEDROOM": bedroom_on.setSFBool(True)
    elif stable_room == "LOUNGE": lounge_on.setSFBool(True)
    elif stable_room == "KITCHEN": kitchen_on.setSFBool(True)

    prev_light_room = stable_room
    light_candidate = None
    light_counter = 0

# ----------------------------------------------------
# OBSTACLE AVOIDANCE
# ----------------------------------------------------
def avoid_obstacle(ps_vals):
    left_front  = max(ps_vals[0], ps_vals[1], ps_vals[2])
    right_front = max(ps_vals[5], ps_vals[6], ps_vals[7])

    base = 2.0
    left = base
    right = base

    if left_front > 120 and left_front > right_front:
        left = 0.8
    elif right_front > 120 and right_front > left_front:
        right = 0.8

    if left_front > 160 and right_front > 160:
        left  = -1.2
        right =  1.2

    return left, right

# ===================== ADD (ROBUST CV: MULTI-CAN + FLOOR SAFE) =====================
GREEN_LOWER = (35, 60, 60)
GREEN_UPPER = (85, 255, 255)

def get_cv_image(cam):
    raw = cam.getImage()
    img = np.frombuffer(raw, np.uint8).reshape((CAM_H, CAM_W, 4))
    return img[:, :, :3]   # BGRA -> BGR

def detect_green_can(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, GREEN_LOWER, GREEN_UPPER)

    # smooth mask (helps far objects + lighting)
    mask = cv2.GaussianBlur(mask, (5, 5), 0)
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best_area = 0
    best_pos = None

    for c in contours:
        area = cv2.contourArea(c)

        # allow far can (small area) but ignore noise + full floor
        if area < 60 or area > 20000:
            continue

        x, y, w, h = cv2.boundingRect(c)

        # reject flat regions (floor)
        aspect_ratio = h / float(w + 1e-5)
        if aspect_ratio < 0.8:
            continue

        # reject bottom-heavy green floor area
        if y > CAM_H * 0.82:
            continue

        # choose best candidate (largest valid)
        if area > best_area:
            best_area = area
            cx = x + w // 2
            if cx < CAM_W * 0.4:
                best_pos = "LEFT"
            elif cx > CAM_W * 0.6:
                best_pos = "RIGHT"
            else:
                best_pos = "CENTER"

    if best_pos is None:
        return None

    return {"area": best_area, "position": best_pos}
# ================================================================================

# ----------------------------------------------------
#  MAP VALUES
# ----------------------------------------------------
ROOM_CENTER = {
    "LOUNGE":  (0.532,  0.053),
    "BEDROOM": (-0.871, 0.770),
    "KITCHEN": (-0.822, -0.864)
}

DOORS = {
    "L_B": (0.333, 0.972),
    "B_K": (-0.872, -0.110)
}

# ----------------------------------------------------
#  ROUTE
# ----------------------------------------------------
ROUTE = [
    ("ROOM", "LOUNGE"),
    ("DOOR", "L_B"),
    ("ROOM", "BEDROOM"),
    ("DOOR", "B_K"),
    ("ROOM", "KITCHEN"),
    ("DOOR", "B_K"),
    ("ROOM", "BEDROOM"),
    ("DOOR", "L_B"),
]

route_index = 0
stop_counter = 0
STOP_STEPS = 20

ROOM_REACH_RADIUS = 0.30
DOOR_REACH_RADIUS = 0.20

MAX_SPEED    = 4.0
KP_DIST      = 1.5
KP_ANGLE     = 3.0
FORWARD_BASE = 2.0

def get_xy():
    x, z, _ = gps.getValues()
    return x, z

def get_heading():
    return imu.getRollPitchYaw()[2]

# ----------------------------------------------------
#  PATH FOLLOWING
# ----------------------------------------------------
def follow_target(current_type, current_name, stable):
    global route_index, stop_counter

    if current_type == "ROOM":
        tx, tz = ROOM_CENTER[current_name]
        radius = ROOM_REACH_RADIUS
    else:
        tx, tz = DOORS[current_name]
        radius = DOOR_REACH_RADIUS

    x, z = get_xy()
    dist = math.sqrt((tx - x)**2 + (tz - z)**2)

    if dist < radius:
        left_motor.setVelocity(0)
        right_motor.setVelocity(0)
        stop_counter += 1
        if stop_counter >= STOP_STEPS:
            route_index = (route_index + 1) % len(ROUTE)
            stop_counter = 0
        return 0.0, 0.0

    if near_wall(ps_vals):
        return avoid_obstacle(ps_vals)

    target_angle = math.atan2(tz - z, tx - x)
    robot_angle  = get_heading()
    angle_error  = math.atan2(math.sin(target_angle - robot_angle),
                              math.cos(target_angle - robot_angle))

    if abs(angle_error) > 0.8:
        turn = 2.0 if angle_error > 0 else -2.0
        return -turn, turn

    forward = FORWARD_BASE + KP_DIST * min(dist, 0.4)
    forward = min(forward, MAX_SPEED)

    turn = KP_ANGLE * angle_error

    return (
        max(min(forward - turn, MAX_SPEED), -MAX_SPEED),
        max(min(forward + turn, MAX_SPEED), -MAX_SPEED)
    )

# ----------------------------------------------------
#  MAIN LOOP
# ----------------------------------------------------
while robot.step(TIME_STEP) != -1:

    gs_vals  = [g.getValue() for g in gs]
    ps_vals  = [p.getValue() for p in ps]
    gps_vals = gps.getValues()

    x, z = get_xy()
    ls_val = ls.getValue() if ls else -1

    raw_room = detect_room(gs_vals)
    override = gps_room_override(x, z)
    stable   = get_stable_room(raw_room, override)

    if freeze_if_bedroom_wall(ps_vals, gps_vals):
        stable = "BEDROOM"
        room_change_counter = 0

    update_lights(stable)

    current_type, current_name = ROUTE[route_index]
    left_speed, right_speed = follow_target(current_type, current_name, stable)

    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
    current_type, current_name = ROUTE[route_index]

    if current_type == "ROOM":
       tx, tz = ROOM_CENTER[current_name]
    else:
       tx, tz = DOORS[current_name]


    # ===================== ADD (CV CHECK) =====================
    frame = get_cv_image(camera)
    green_can = detect_green_can(frame)
    # =========================================================

    obstacle = "YES" if max(ps_vals) > 120 else "NO"

    print(
        f"ROOM={stable} | "
        f"LIGHT_ON={prev_light_room} | "
        f"GREEN_CAN={'YES' if green_can else 'NO'} | "
        f"OBSTACLE={obstacle} | "
        f"GPS=({x:.3f},{z:.3f}) | "
        f"GS={['%.1f'%v for v in gs_vals]} | "
        f"PS={['%.1f'%v for v in ps_vals]} | "
        f"LS={ls_val:.1f} | "
        f"TARGET={current_type}:{current_name} | "
        f"TARGET_POS=({tx:.2f},{tz:.2f}) | "

    )
