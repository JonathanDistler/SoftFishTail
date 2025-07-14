import serial
import time
from datetime import datetime
import numpy as np
import cv2
import os

# Dynamixel control setup
from dxlControlPath import relativeDir
relativeDir('../')
from dxlSetup.portAndPackets import openPortObject, closePortObject
from dxlSetup.groupSyncFuncs import *
from dxlSetup.XL430 import XL430

#again, this requires MattFern's Dynamixel class, but this is intended to live stream the calibrated camera settings for the fish force test

# ---------------------- Camera Calibration ----------------------
#These values don't work super well with the fish at the moment. Would need to remove the test-rig in order to accurately calibrate the camera
#to the fish's depth
mtx = np.array([[794.06933744, 0., 931.41508085],
                [0., 794.10552238, 539.85862057],
                [0., 0., 1.]])
dist = np.array([-2.75729559e-01,  1.04389635e-01, -4.49111933e-04, -7.43910505e-05,
                 -2.53113830e-02])

# ---------------------- Configuration ----------------------
desiredFreq = 2                  # Hz (cycle frequency)
commandTime = 0.08               # seconds between servo commands
numCycles = 30                   # number of movement cycles
saveData = True
motorVel = 235                   # DXL velocity
gearRatio = 5.4
motorTicks = 4096
stepSize = motorTicks * (desiredFreq / gearRatio) * commandTime
all_steps = int((numCycles / desiredFreq) / commandTime)
scale = 3                        # Frame scale factor

# ---------------------- Initialize Data ----------------------
position_data = np.zeros((all_steps + 1, 1))
load_data = np.zeros((all_steps + 1, 1))
vel_data = np.zeros((all_steps + 1, 1))
time_data = np.zeros((all_steps + 1, 1))
goal_pos_data = np.zeros((all_steps + 2, 1))

# ---------------------- Time & File Setup ----------------------
output_dir = r"C:\Users\15405\OneDrive\Desktop\Career\ETHZ\ETHZ Work\HardwareOutput"
os.makedirs(output_dir, exist_ok=True)
global_start_time = datetime.now().strftime("%m-%d_%H-%M-%S")
video_filename = os.path.join(output_dir, f'output_{global_start_time}.avi')
csv_filename = os.path.join(output_dir, f'force_readings_{global_start_time}.csv')

# ---------------------- Open Camera ----------------------
camera = cv2.VideoCapture(0)
ret, frame = camera.read()
if not ret:
    print("Error: Could not capture initial frame.")
    exit()

# Initialize video writer later
out_video = None

# ---------------------- Open Serial ----------------------
ser = serial.Serial('COM9', 9600)
start = time.time()

# ---------------------- Open Dynamixel ----------------------
openPortObject()
servo = XL430(id=1, zeroPos=0)
servo.torqueEnable()
servo.goalPos = servo.readPos()
time.sleep(2)

print("Recording... Press 'q' to stop.")

# ---------------------- Start Trial ----------------------
with open(csv_filename, 'w') as f:
    f.write(f"Frequency-Intended {desiredFreq}, Start Time {global_start_time}, Number of cycle {numCycles}, Gear Ratio {gearRatio}\n")
    f.write("Time (s), Force, GoalPos, ServoPos, Load, Velocity\n")

    try:
        initial_time = time.time()
        position_data[0] = servo.readPos()
        vel_data[0] = servo.readVel()
        load_data[0] = servo.readLoad()
        goal_pos_data[0:2] = servo.goalPos
        time_data[0] = initial_time

        for step in range(all_steps):
            loop_start = time.time()

            # ---------- Frame Capture and Undistortion ----------
            ret, frame = camera.read()
            if not ret:
                print("Error: Failed to capture frame.")
                break

            h, w = frame.shape[:2]
            newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
            dst = cv2.undistort(frame, mtx, dist, None, newcameramtx)
            x, y, w_roi, h_roi = roi
            dst = dst[y:y+h_roi, x:x+w_roi]
            resized = cv2.resize(dst, None, fx=scale, fy=scale)

            # ---------- Initialize Video Writer ----------
            if out_video is None:
                height, width = resized.shape[:2]
                output_size = (width, height)
                fourcc = cv2.VideoWriter_fourcc(*'XVID')
                out_video = cv2.VideoWriter(video_filename, fourcc, 1/commandTime, output_size)
                if not out_video.isOpened():
                    print("Error: Could not open video writer.")
                    exit()

            if (resized.shape[1], resized.shape[0]) != output_size:
                print("Frame size mismatch. Skipping frame.")
                continue

            # ---------- Move Servo ----------
            servo.goalPos += stepSize
            goalPosition = servo.goalPos
            servo.move(goal_pos=int(round(goalPosition)), prof_vel=motorVel)

            # ---------- Read Sensors ----------
            position = servo.readPos()
            velocity = servo.readVel()
            load = servo.readLoad()
            timestamp = time.time() - start

            position_data[step + 1] = position
            vel_data[step + 1] = velocity
            load_data[step + 1] = load
            goal_pos_data[step + 2] = goalPosition
            time_data[step + 1] = time.time()

            # ---------- Read Force Sensor ----------
            line = ser.readline().decode('utf-8', errors='ignore').strip()

            # ---------- Overlay Info ----------
            current_time = datetime.now().strftime("%m-%d %H:%M:%S")
            cv2.putText(resized, f"Start: {global_start_time} | Now: {current_time}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(resized, f"Force: {line}", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(resized, f"Motor Position: {int(goalPosition)}", (10, 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2, cv2.LINE_AA)

            # ---------- Save Frame + Data ----------
            out_video.write(resized)
            cv2.imshow('Recording', resized)
            f.write(f"{timestamp:.2f},{line},{int(goalPosition)},{position},{load},{velocity}\n")

            # Break if 'q' pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("Recording stopped by user.")
                break

            # Maintain timing
            loop_duration = time.time() - loop_start
            if loop_duration < commandTime:
                time.sleep(commandTime - loop_duration)

    except KeyboardInterrupt:
        print("Recording interrupted by user.")

# ---------------------- Cleanup ----------------------
camera.release()
if out_video:
    out_video.release()
cv2.destroyAllWindows()
closePortObject()

print("Done. Files saved:")
print(f"  → Video: {video_filename}")
print(f"  → Force CSV: {csv_filename}")


#get updated parameters from chess board for underwater table
