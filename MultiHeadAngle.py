# !/usr/bin/env python3
#code almost entirely from Mike Yan Michelis

"""Track bounding boxes within video.

The first box is a rigidly mounted plate at the middle of the screen and the second box is the head of the fish. This script iterates over all different frequency videos and measures the angles of the head with respect to a 
rigidly mounted box. It also removes redundancy of having to specify/hard code end frame and the fps

"""

import os
import time
import click
import cv2
import numpy as np
from datetime import datetime
import math
import csv
import matplotlib.pyplot as plt

# Function to adjust contrast
def adjust_contrast(frame, alpha, beta):
    # alpha: Contrast control (1.0-3.0), beta: Brightness control (0-100)
    return cv2.convertScaleAbs(frame, alpha=alpha, beta=beta)


def track_markers(filepath: str, num_boxes: int, start_frame: int,  freq: float):
    folder=f"C:/Users/15405/OneDrive/Desktop/Career/ETHZ/ETHZ Work/HardwareOutput/Real-to-Sim-tests"
    filepath=f"C:/Users/15405/OneDrive/Desktop/Career/ETHZ/ETHZ Work/HardwareOutput/Real-to-Sim-tests/{freq}.mp4"
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    end_frame=start_frame+total_frames
    
    cap = cv2.VideoCapture(filepath)

    fps = cap.get(cv2.CAP_PROP_FPS)

    framenum = 0
    print("Select bounding boxes in TAIL-TO-HEAD order.")
    while cap.isOpened():
        ret, frame = cap.read()
        framenum += 1
        if framenum < start_frame:
            continue
        if not ret:
            print("Error: Could not read frame.")
            return
        
        height, width = frame.shape[:2]
        y_center = height // 2
        x_center = width // 2
        start_point = (0, y_center)
        end_point = (width - 1, y_center)

        bboxes = []
        for i in range(num_boxes):
            roi = cv2.selectROI(f"Select box {i+1} (tail to head)", frame)
            bboxes.append(roi)
        break
    cap.release()
    cv2.destroyAllWindows()

    # Initialize MultiTracker
    trackers = cv2.legacy.MultiTracker_create()
    #trackers = cv2.MultiTracker_create()
    for box in bboxes:
        trackers.add(cv2.legacy.TrackerCSRT_create(), frame, box)

    cap = cv2.VideoCapture(filepath)
    fps = cap.get(cv2.CAP_PROP_FPS)
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")

    
    cut_movie = cv2.VideoWriter(f"{folder}/cut_movie_{freq}.mp4", fourcc, fps, (frame.shape[1], frame.shape[0]))
    tracked_movie = cv2.VideoWriter(f"{folder}/CV_movie_{freq}.mp4", fourcc, fps, (frame.shape[1], frame.shape[0]))

    #arrays for the markers, the metrics of the tail segments relative to the head, tail segments relative to each other, and the total tail-segment data
    markers = []
    all_rel_metrics = []
    all_rel_metrics_2=[]
    all_abs_metrics=[]
    framenum = 0

    while cap.isOpened():
        ret, frame = cap.read()
        framenum += 1
        if not ret or framenum > end_frame:
            break
        #draws line across the center of the screen
        cv2.line(frame, start_point, end_point, color=(255, 255, 255), thickness=1)
        if framenum < start_frame:
            continue
        
        #processes all of the images to make them easier to "read" for the CV algorithm, had been 1-75
        alpha=1
        beta=45
        frame = adjust_contrast(frame, alpha, beta)
        cut_movie.write(frame)

        height, width = frame.shape[:2]
        y_center = height // 2
        start_point = (0, y_center)
        end_point = (width - 1, y_center)
        cv2.line(frame, start_point, end_point, (255, 255, 255), 1)

        found, bboxes_new = trackers.update(frame)
        if not found:
            cv2.putText(frame, "Tracking failed", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)

        curr_markers = []
        for i, box in enumerate(bboxes_new):
            x, y, w, h = box
            cx = int(x + w / 2)
            cy = int(y + h / 2)
            curr_markers.append((cx, cy))
            # Draw bounding boxes
            cv2.rectangle(frame, (int(x), int(y)), (int(x + w), int(y + h)), (0, 255, 0), 2)

        # Draws tail as polyline
        if len(curr_markers) >= 2:
            points = np.array(curr_markers, dtype=np.int32)
            cv2.polylines(frame, [points], isClosed=False, color=(0, 255, 255), thickness=2)

        markers.append(np.array(curr_markers).flatten())

        # Reference is head (last box in tail-to-head list)
        head_center = np.array(curr_markers[-1])
        rel_metrics = []
        #measures angle, distance, etc. via the last rectangular object ('the head')
        for i in range(num_boxes - 1):
            pt = np.array(curr_markers[i])
            dx = pt[0] - head_center[0]
            dy = pt[1] - head_center[1]
            distance = np.sqrt(dx**2 + dy**2)
            angle = math.atan2(dy, dx) * 180 / np.pi
            #if (angle>0):
                #angle=angle-180
            #else:
                #angle=angle+180
            rel_metrics.append([framenum, i, dx, dy, distance, angle])

        #adds to a new array
        all_rel_metrics.extend(rel_metrics)

        tracked_movie.write(frame)

    cap.release()
    cut_movie.release()
    tracked_movie.release()
    cv2.destroyAllWindows()


    np.savetxt(f"{folder}/markers_{freq}.csv", markers, delimiter=",")
    with open(f"{folder}/HeadSegement_rel_Stationary_{freq}.csv", mode='w', newline='') as file:
        writer = csv.writer(file)
        #adds meta data to the top of the CSV
        writer.writerow(["Number of Boxes","Total Time","Start Frame","End Frame","Video Path","Alpha","Beta"])
        writer.writerow([num_boxes,(end_frame-start_frame)*(1/fps),start_frame,end_frame,filepath,alpha,beta])

        #adds real formatting for data
        writer.writerow(["Time (s)", "Box Index", "dx", "dy", "Distance", "Angle"])
        for framenum,i,dx,dy,distance,angle in all_rel_metrics:
            if i==1:
                value="Head"
            else:
                value="Stationary"
            t=(1/fps)*framenum
            #times.append(t)
            writer.writerow([t,value,dx,dy,distance,angle])


if __name__ == "__main__":
    #for freq in np.arange(0.6, 2.1, 0.1): 
    for freq in np.arange(0.6, .9, 0.2): 
        freq = round(freq, 1)

        track_markers(

            filepath="",         # avoids having to use commands in prompt shell
            num_boxes=2,
            start_frame=0,
            freq=freq
        )

