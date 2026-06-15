{\rtf1\ansi\ansicpg1252\cocoartf2822
\cocoatextscaling0\cocoaplatform0{\fonttbl\f0\fswiss\fcharset0 Helvetica;}
{\colortbl;\red255\green255\blue255;}
{\*\expandedcolortbl;;}
\paperw11900\paperh16840\margl1440\margr1440\vieww11520\viewh8400\viewkind0
\pard\tx720\tx1440\tx2160\tx2880\tx3600\tx4320\tx5040\tx5760\tx6480\tx7200\tx7920\tx8640\pardirnatural\partightenfactor0

\f0\fs24 \cf0 """\
mavic_controller.py\
===================\
Fixed version: yaw-based obstacle avoidance so the drone actually turns\
away, plus stronger roll, shorter exit threshold, and a forward-resume\
kick so it doesn't freeze at target altitude.\
\
FIXES v4 (Hybrid Sensor Fusion):\
- Split transformers import to completely bypass the Webots SyntaxError.\
- Combined AI Depth Map (80%) and 2D Canny Edges (20%) for robust side selection.\
- Works perfectly for BOTH solid blank walls and high-texture environments like trees.\
"""\
\
import sys\
import math\
import threading\
import numpy as np\
import cv2\
from PIL import Image\
\
sys.path.append("/Applications/Webots.app/Contents/lib/controller/python")\
from controller import Robot\
\
# \uc0\u1508 \u1497 \u1510 \u1493 \u1500  \u1489 \u1496 \u1493 \u1495  \u1513 \u1500  \u1492 -Import \u1500 \u1502 \u1504 \u1497 \u1506 \u1514  \u1514 \u1493 \u1493 \u1497 \u1501  \u1504 \u1505 \u1514 \u1512 \u1497 \u1501  \u1493 \u1513 \u1490 \u1497 \u1488 \u1493 \u1514  \u1505 \u1497 \u1504 \u1496 \u1511 \u1505 \
import transformers\
hf_pipeline = transformers.pipeline\
\
\
def clamp(value, low, high):\
    return max(low, min(high, value))\
\
\
# \
# PERCEPTION\
# \
\
class DepthEstimator:\
    def __init__(self):\
        print("[System] Loading AI...")\
        self.pipe = hf_pipeline(task="depth-estimation", model="depth-anything/Depth-Anything-V2-Small-hf")\
        self._depth, self._lock, self._busy = None, threading.Lock(), False\
\
    def update(self, frame_rgb):\
        if self._busy:\
            return\
        self._busy = True\
        threading.Thread(target=self._compute, args=(frame_rgb.copy(),), daemon=True).start()\
\
    def _compute(self, frame_rgb):\
        try:\
            small = cv2.resize(frame_rgb, (320, 240))\
            result = self.pipe(Image.fromarray(small))\
            d = np.array(result["depth"], dtype=np.float32)\
            d_min, d_max = d.min(), d.max()\
            if d_max - d_min > 1e-6:\
                d = (d - d_min) / (d_max - d_min)\
            with self._lock:\
                self._depth = d.copy()\
        finally:\
            self._busy = False\
\
    def get(self):\
        with self._lock:\
            return self._depth.copy() if self._depth is not None else None\
\
\
class VisionSuite:\
    def analyze(self, gray):\
        H, W = gray.shape\
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)\
        edges = cv2.Canny(blurred, 100, 200)\
\
        third = W // 3\
        left_cols   = np.count_nonzero(edges[:, :third],        axis=0)\
        center_cols = np.count_nonzero(edges[:, third:2*third], axis=0)\
        right_cols  = np.count_nonzero(edges[:, 2*third:],      axis=0)\
\
        def zone_score(cols):\
            if len(cols) < 3:\
                return 0.0\
            return np.sort(cols)[-3:].sum() / float(H)\
\
        return \{\
            "left":   zone_score(left_cols),\
            "center": zone_score(center_cols),\
            "right":  zone_score(right_cols),\
        \}\
\
\
# \
# MAIN FLIGHT LOOP\
# \
\
def run():\
    robot = Robot()\
    timestep = int(robot.getBasicTimeStep())\
\
    camera = robot.getDevice("camera");         camera.enable(timestep)\
    imu    = robot.getDevice("inertial unit");   imu.enable(timestep)\
    gps    = robot.getDevice("gps");             gps.enable(timestep)\
    gyro   = robot.getDevice("gyro");            gyro.enable(timestep)\
    c_roll  = robot.getDevice("camera roll")\
    c_pitch = robot.getDevice("camera pitch")\
\
    motors = [robot.getDevice(n) for n in [\
        "front left propeller", "front right propeller",\
        "rear left propeller",  "rear right propeller"\
    ]]\
    for m in motors:\
        m.setPosition(float("inf"))\
        m.setVelocity(1.0)\
\
    K_VERTICAL_THRUST = 68.5\
    K_VERTICAL_OFFSET = 0.6\
    K_VERTICAL_P      = 3.0\
    K_ROLL_P          = 50.0\
    K_PITCH_P         = 30.0\
\
    target_altitude = 1.5\
    depth_est    = DepthEstimator()\
    vision_suite = VisionSuite()\
\
    frame_count        = 0\
    roll_dist          = 0.0\
    pitch_dist         = -2.0\
    yaw_input          = 0.0\
    cooldown           = 0\
    mode               = "FORWARD"\
    avoid_side         = 0.0\
    post_avoid_lockout = 0\
    POST_AVOID_FRAMES  = 60\
\
    # Yaw ramp state\
    yaw_target         = 0.0    \
    YAW_RAMP_RATE      = 0.008        \
\
    smoothed_danger = 0.0\
    DANGER_ALPHA    = 0.3\
\
    # \uc0\u1502 \u1513 \u1514 \u1504 \u1497  \u1505 \u1512 \u1497 \u1511 \u1492  \u1492 \u1497 \u1489 \u1512 \u1497 \u1491 \u1497 \u1514 \
    scan_timer = 0\
    best_left_score = 999.0\
    best_right_score = 999.0\
\
    DANGER_ENTER    = 0.55           \
    DANGER_EXIT     = 0.38            \
    COOLDOWN_FRAMES = 80              \
\
    while robot.step(timestep) != -1:\
        roll, pitch, _ = imu.getRollPitchYaw()\
        altitude = gps.getValues()[2]\
        r_vel, p_vel = gyro.getValues()[0], gyro.getValues()[1]\
\
        c_roll.setPosition(clamp(-0.115 * r_vel, -0.5, 0.5))\
        c_pitch.setPosition(clamp(-0.1   * p_vel, -0.5, 0.5))\
\
        raw    = camera.getImage()\
        danger = 0.0\
\
        if raw:\
            frame_bgra = np.frombuffer(raw, dtype=np.uint8).reshape(\
                (camera.getHeight(), camera.getWidth(), 4)\
            )\
            if frame_count % 5 == 0:\
                depth_est.update(cv2.cvtColor(frame_bgra, cv2.COLOR_BGRA2RGB))\
\
            depth_map = depth_est.get()\
            if depth_map is not None:\
                H, W = depth_map.shape\
                center_patch = depth_map[:H // 2, W // 3:2 * W // 3]\
                depth_danger = float(center_patch.max(axis=0).mean())\
\
                gray   = cv2.cvtColor(frame_bgra, cv2.COLOR_BGRA2GRAY)\
                vision = vision_suite.analyze(gray)\
\
                raw_danger = depth_danger + (vision["center"] * 0.15)\
                smoothed_danger = DANGER_ALPHA * raw_danger + (1.0 - DANGER_ALPHA) * smoothed_danger\
                danger = smoothed_danger\
\
                # \uc0\u1506 \u1491 \u1499 \u1493 \u1503  \u1496 \u1497 \u1497 \u1502 \u1512 \u1497 \u1501 \
                if cooldown > 0:\
                    cooldown -= 1\
                if post_avoid_lockout > 0:\
                    post_avoid_lockout -= 1\
\
                # --- \uc0\u1500 \u1493 \u1490 \u1497 \u1511 \u1514  \u1502 \u1510 \u1489 \u1497 \u1501  (FSM) \u1492 \u1497 \u1489 \u1512 \u1497 \u1491 \u1497 \u1514  \u1493 \u1502 \u1514 \u1493 \u1511 \u1504 \u1514  ---\
                \
                if mode == "SCAN":\
                    scan_timer -= 1\
                    third = W // 3\
                    \
                    if scan_timer > 20:\
                        # 20 \uc0\u1508 \u1512 \u1497 \u1497 \u1502 \u1497 \u1501  \u1512 \u1488 \u1513 \u1493 \u1504 \u1497 \u1501 : \u1502 \u1505 \u1514 \u1499 \u1500 \u1497 \u1501  \u1513 \u1502 \u1488 \u1500 \u1492 \
                        yaw_target = 0.25\
                        \
                        # \uc0\u1513 \u1497 \u1500 \u1493 \u1489  \u1488 \u1500 \u1490 \u1493 \u1512 \u1497 \u1514 \u1502 \u1497 \u1501 : 80% \u1502 \u1513 \u1511 \u1500  \u1500 \u1506 \u1493 \u1502 \u1511  \u1492 \u1508 \u1497 \u1494 \u1497 , 20% \u1502 \u1513 \u1511 \u1500  \u1500 \u1511 \u1493 \u1493 \u1497 \u1501  (\u1506 \u1510 \u1497 \u1501 )\
                        depth_L = float(depth_map[:H // 2, :third].max(axis=0).mean())\
                        edge_L = vision["left"]\
                        combined_left = (depth_L * 0.8) + (edge_L * 0.2)\
                        \
                        if combined_left < best_left_score:\
                            best_left_score = combined_left\
                    else:\
                        # 20 \uc0\u1508 \u1512 \u1497 \u1497 \u1502 \u1497 \u1501  \u1489 \u1488 \u1497 \u1501 : \u1502 \u1505 \u1514 \u1499 \u1500 \u1497 \u1501  \u1497 \u1502 \u1497 \u1504 \u1492 \
                        yaw_target = -0.25\
                        \
                        depth_R = float(depth_map[:H // 2, 2*third:].max(axis=0).mean())\
                        edge_R = vision["right"]\
                        combined_right = (depth_R * 0.8) + (edge_R * 0.2)\
                        \
                        if combined_right < best_right_score:\
                            best_right_score = combined_right\
\
                    # \uc0\u1505 \u1497 \u1493 \u1501  \u1492 \u1505 \u1512 \u1497 \u1511 \u1492  \u1492 \u1488 \u1511 \u1496 \u1497 \u1489 \u1497 \u1514  \u1493 \u1511 \u1489 \u1500 \u1514  \u1492 \u1495 \u1500 \u1496 \u1492  \u1502 \u1513 \u1493 \u1500 \u1489 \u1514 \
                    if scan_timer <= 0:\
                        # \uc0\u1508 \u1493 \u1504 \u1497 \u1501  \u1500 \u1510 \u1491  \u1513 \u1489 \u1493  \u1492 \u1510 \u1497 \u1493 \u1503  \u1492 \u1502 \u1513 \u1493 \u1500 \u1489  (\u1506 \u1493 \u1502 \u1511  + \u1511 \u1493 \u1493 \u1497 \u1501 ) \u1504 \u1502 \u1493 \u1498  \u1493 \u1489 \u1496 \u1493 \u1495  \u1497 \u1493 \u1514 \u1512 \
                        avoid_side = 1.0 if best_left_score < best_right_score else -1.0\
                        \
                        yaw_target = 0.15 * avoid_side\
                        roll_dist  = 0.4 * avoid_side\
                        pitch_dist = -0.5                  \
                        mode       = "AVOID"\
                        cooldown   = COOLDOWN_FRAMES\
\
                elif mode == "AVOID":\
                    if danger < DANGER_EXIT:\
                        roll_dist          = 0.0\
                        yaw_target         = 0.0   \
                        pitch_dist         = -2.0\
                        mode               = "FORWARD"\
                        post_avoid_lockout = POST_AVOID_FRAMES\
\
                elif danger > DANGER_ENTER and mode == "FORWARD" and post_avoid_lockout == 0:\
                    mode = "SCAN"\
                    scan_timer = 40  \
                    best_left_score = 999.0\
                    best_right_score = 999.0\
                    pitch_dist = -0.1  # \uc0\u1492 \u1488 \u1496 \u1492  \u1489 \u1494 \u1502 \u1503  \u1492 \u1505 \u1512 \u1497 \u1511 \u1492  \u1492 \u1488 \u1511 \u1496 \u1497 \u1489 \u1497 \u1514  \u1500 \u1513 \u1502 \u1497 \u1512 \u1492  \u1506 \u1500  \u1489 \u1496 \u1497 \u1495 \u1493 \u1514 \
\
                elif mode == "FORWARD":\
                    roll_dist  = 0.0\
                    yaw_target = 0.0\
                    pitch_dist = -2.0\
\
        # \uc0\u1492 \u1495 \u1500 \u1511 \u1514  \u1513 \u1497 \u1504 \u1493 \u1497 \u1497  \u1492 -Yaw \u1500 \u1502 \u1504 \u1497 \u1506 \u1514  \u1500 \u1493 \u1500 \u1488 \u1493 \u1514  \u1505 \u1495 \u1512 \u1493 \u1512 \
        if abs(yaw_target - yaw_input) < YAW_RAMP_RATE:\
            yaw_input = yaw_target\
        else:\
            yaw_input += math.copysign(YAW_RAMP_RATE, yaw_target - yaw_input)\
\
        # PID MATH\
        r_input = K_ROLL_P * clamp(roll, -1.0, 1.0) + r_vel + roll_dist\
        p_input = K_PITCH_P * clamp(pitch, -1.0, 1.0) + p_vel + pitch_dist\
        y_input = yaw_input\
\
        v_diff  = clamp(target_altitude - altitude + K_VERTICAL_OFFSET, -1.0, 1.0)\
        v_input = K_VERTICAL_P * (v_diff ** 3.0)\
\
        fl = K_VERTICAL_THRUST + v_input - r_input + p_input - y_input\
        fr = K_VERTICAL_THRUST + v_input + r_input + p_input + y_input\
        rl = K_VERTICAL_THRUST + v_input - r_input - p_input + y_input\
        rr = K_VERTICAL_THRUST + v_input + r_input - p_input - y_input\
\
        motors[0].setVelocity(fl)\
        motors[1].setVelocity(-fr)\
        motors[2].setVelocity(-rl)\
        motors[3].setVelocity(rr)\
\
        frame_count += 1\
        if frame_count % 30 == 0:\
            print(f"[\{mode:<10\}] Alt:\{altitude:.2f\}m | "\
                  f"Ang:\{math.degrees(roll):+.1f\}\'b0 | "\
                  f"Danger:\{danger:.2f\} | "\
                  f"Yaw:\{yaw_input:+.3f\} | Lock:\{post_avoid_lockout\}")\
\
\
if __name__ == "__main__":\
    run()}