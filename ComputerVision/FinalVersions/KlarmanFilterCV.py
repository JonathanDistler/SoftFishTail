"""
Track bounding boxes within video.

Notes:
- OpenCV version: opencv-contrib-python 4.5.2.52
- 2 boxes: first stationary, second = fish head
- Kalman filter used for smoothing and prediction
"""

import cv2
import numpy as np
import math
import csv
from filterpy.kalman import KalmanFilter

alpha = 1.2
beta = 95

def adjust_contrast(frame, alpha, beta):
    return cv2.convertScaleAbs(frame, alpha=alpha, beta=beta)

def create_kalman_filter():
    kf = KalmanFilter(dim_x=4, dim_z=2)
    kf.x = np.zeros(4)
    kf.P *= 1000.
    kf.F = np.array([[1,0,1,0],
                     [0,1,0,1],
                     [0,0,1,0],
                     [0,0,0,1]])
    kf.H = np.array([[1,0,0,0],
                     [0,1,0,0]])
    kf.R = np.array([[10,0],
                     [0,10]])
    kf.Q = np.eye(4)
    return kf

def track_markers(filepath: str, num_boxes: int, start_frame: int, freq: float):
    folder = r"C:\Users\jonat\Downloads"

    cap = cv2.VideoCapture(filepath)
    if not cap.isOpened():
        print(f"Cannot open video: {filepath}")
        return

    print("Select bounding boxes in TAIL-TO-HEAD order.")
    ret, frame = cap.read()
    if not ret:
        print("Cannot read first frame")
        return
    
    frame = adjust_contrast(frame, alpha, beta)

    bboxes = []
    for i in range(num_boxes):
        roi = cv2.selectROI(f"Select box {i+1} (tail to head)", frame)
        print("ROI:", roi)
        bboxes.append(roi)
    cv2.destroyAllWindows()

    # Initialize MultiTracker
    trackers = cv2.legacy.MultiTracker_create()
    for box in bboxes:
        trackers.add(cv2.legacy.TrackerCSRT_create(), frame, box)

    # Initialize Kalman filters
    kalman_filters = [create_kalman_filter() for _ in range(num_boxes)]

    fps = cap.get(cv2.CAP_PROP_FPS)
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    cut_movie = cv2.VideoWriter(f"{folder}/cut_movie_{freq}_HZ.avi", fourcc, fps, (frame.shape[1], frame.shape[0]))
    tracked_movie = cv2.VideoWriter(f"{folder}/CV_movie_{freq}_HZ.avi", fourcc, fps, (frame.shape[1], frame.shape[0]))

    markers = []
    all_rel_metrics = []

    framenum = 0
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            print("End of video or error reading frame.")
            break

        framenum += 1
        if framenum < start_frame:
            continue

        frame = adjust_contrast(frame, alpha, beta)
        height, width = frame.shape[:2]
        start_point = (0, height // 2)
        end_point = (width - 1, height // 2)
        cv2.line(frame, start_point, end_point, (255, 255, 255), 1)

        found, bboxes_new = trackers.update(frame)
        curr_markers = []

        for i in range(num_boxes):
            if found:
                x, y, w, h = bboxes_new[i]
                cx = int(x + w / 2)
                cy = int(y + h / 2)
                kalman_filters[i].predict()
                kalman_filters[i].update([cx, cy])
                pred_x, pred_y = kalman_filters[i].x[:2]
                cv2.rectangle(frame, (int(x), int(y)), (int(x+w), int(y+h)), (0,255,0), 2)
            else:
                kalman_filters[i].predict()
                pred_x, pred_y = kalman_filters[i].x[:2]

            curr_markers.append((int(pred_x), int(pred_y)))
            cv2.circle(frame, (int(pred_x), int(pred_y)), 5, (0,255,0), -1)

        # Draw polyline
        if len(curr_markers) >= 2:
            points = np.array(curr_markers, dtype=np.int32)
            cv2.polylines(frame, [points], isClosed=False, color=(0,255,255), thickness=2)

        markers.append(np.array(curr_markers).flatten())

        # Relative metrics using last box as head
        head_center = np.array(curr_markers[-1])
        for i in range(num_boxes - 1):
            pt = np.array(curr_markers[i])
            dx = pt[0] - head_center[0]
            dy = pt[1] - head_center[1]
            distance = np.sqrt(dx**2 + dy**2)
            angle = math.atan2(dy, dx) * 180 / np.pi
            all_rel_metrics.append([framenum, i, dx, dy, distance, angle])

        cut_movie.write(frame)
        tracked_movie.write(frame)

        # Optional display
        cv2.imshow("Tracking", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cut_movie.release()
    tracked_movie.release()
    cv2.destroyAllWindows()

    # Pad markers for CSV
    max_len = num_boxes*2
    padded_markers = []
    for m in markers:
        arr = np.array(m).flatten()
        if len(arr) < max_len:
            arr = np.pad(arr, (0, max_len-len(arr)), constant_values=np.nan)
        padded_markers.append(arr)

    np.savetxt(f"{folder}/markers_{freq}.csv", padded_markers, delimiter=",")

    with open(f"{folder}/HeadSegment_rel_Stationary_{freq}.csv", "w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["Number of Boxes","Start Frame","End Frame","Video Path","Alpha","Beta"])
        writer.writerow([num_boxes, start_frame, framenum, filepath, alpha, beta])
        writer.writerow(["frame","dx","dy","Distance","Angle"])
        for framenum, i, dx, dy, distance, angle in all_rel_metrics:
            writer.writerow([framenum, dx, dy, distance, angle])

def main():
    filepath = r"C:\Users\jonat\Downloads\20250805_1546_f0.5Hz_film_test_data.avi"
    start_index = filepath.find("f", 20)
    freq = float(filepath[start_index+1:start_index+4])
    track_markers(filepath=filepath, num_boxes=2, start_frame=30, freq=freq)

if __name__ == "__main__":
    main()
