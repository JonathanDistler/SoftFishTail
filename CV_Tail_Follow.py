# !/usr/bin/env python3
#code almost entirely from Mike Yan Michelis
#Working on developing a function to compute the angle between the fish_head (first bounding box) and a stationary bounding box:
#could work on making stationary bounding box permanent to the frame 

"""Track bounding boxes within video.

Usage: python3 track_markers.py -f <filepath> -n <num_boxes> -s <start_frame> -e <end_frame>

Example: python3 track_markers.py -f ~/Downloads/tracking/vid.mp4 -n 2 -s 10 -e 50

Script for tracking N manually chosen bounding boxes within video. Point this script to the video file you would like to track and choose how many N bounding boxes are desired. These bounding box centers (markers) are stored in a CSV file afterwards. Two videos are created as well, one is the original video cut to [start_frame, end_frame], and the other is with the tracking bounding boxes displayed. If you for some reason desire to quite the tracking earlier than end_frame, you can press q to exit out.

Note: OpenCV installation can sometimes have trouble with cv2.legacy.MultiTracker_create(), the version that worked is:
opencv-contrib-python 4.5.2.52
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


@click.command()
@click.option(
    "--filepath",
    "-f",
    help="Video filepath. Output files will also be stored in its folder.",
    required=True,
)
@click.option("--num_boxes", "-n", default=2, help="Number of bounding boxes.")
@click.option(
    "--start_frame", "-s", default=0, help="Relevant starting frame of video."
)
@click.option(
    "--end_frame", "-e", default=100, help="Relevant ending frame of video."
)

def track_markers(filepath: str, num_boxes: int, start_frame: int, end_frame: int):
    folder = os.path.dirname(filepath)
    cap = cv2.VideoCapture(filepath)

    #formatted datetime without the second
    formatted_datetime = datetime.now().strftime("%Y-%m-%d_%H-%M")
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
    for box in bboxes:
        trackers.add(cv2.legacy.TrackerCSRT_create(), frame, box)

    cap = cv2.VideoCapture(filepath)
    fps = cap.get(cv2.CAP_PROP_FPS)
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")

    cut_movie = cv2.VideoWriter(f"{folder}/cut_video.mp4", fourcc, fps, (frame.shape[1], frame.shape[0]))
    tracked_movie = cv2.VideoWriter(f"{folder}/tracked_video_{formatted_datetime}.mp4", fourcc, fps, (frame.shape[1], frame.shape[0]))

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
        rel_metrics_2=[]
        abs_metrics=[]
        #measures angle, distance, etc. via the last rectangular object ('the head')
        for i in range(num_boxes - 1):
            pt = np.array(curr_markers[i])
            dx = pt[0] - head_center[0]
            dy = pt[1] - head_center[1]
            distance = np.sqrt(dx**2 + dy**2)
            angle = math.atan2(dy, dx) * 180 / np.pi
            if (angle>0):
                angle=angle-180
            else:
                angle=angle+180
            rel_metrics.append([framenum, i, dx, dy, distance, angle])
        #measures the angle, distance, etc. via the next rectangular object ('next tail segment')
        total_angle_sum=0
        for i in range(num_boxes - 1):
            pt_1 = np.array(curr_markers[i])
            pt_2=np.array(curr_markers[i+1])
            #not dividing by 2, because unlike CV_test, the rectangles are drawn about their mid-point
            dx = (pt_1[0] - pt_2[0])
            dy = (pt_1[1] - pt_2[1])
            distance = np.sqrt(dx**2 + dy**2)
            angle = math.atan2(dy, dx) * 180 / np.pi
            if (angle>0):
                angle=angle-180
            else:
                angle=angle+180

            total_angle_sum+=angle    

            rel_metrics_2.append([framenum, i, dx, dy, distance, angle])
        abs_metrics.append([framenum,total_angle_sum])

        #adds to a new array
        all_rel_metrics.extend(rel_metrics)
        all_rel_metrics_2.extend(rel_metrics_2)

        all_abs_metrics.extend(abs_metrics)
        tracked_movie.write(frame)
        cv2.imshow("Tracker (press Q to exit)", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            print("Exited early.")
            break

    cap.release()
    cut_movie.release()
    tracked_movie.release()
    cv2.destroyAllWindows()


    fps=30 #from Matthew's phone information, this needs to be hardcoded in
    #need to change up all of the loops to reference s instead of fps 
    #need to figure out the math on the distance 
    #framenum, i, dx, dy, distance, angle

    np.savetxt(f"{folder}/markers.csv", markers, delimiter=",")
    with open(f"{folder}/TailSegments_to_HeadSegments_Metrics.csv", mode='w', newline='') as file:
        writer = csv.writer(file)
        #adds meta data to the top of the CSV
        writer.writerow(["Number of Boxes","Total Time","Start Frame","End Frame","Video Path","Alpha","Beta"])
        writer.writerow([num_boxes,(end_frame-start_frame)*(1/fps),start_frame,end_frame,filepath,alpha,beta])

        #adds real formatting for data
        writer.writerow(["Time (s)", "Box Index", "dx", "dy", "Distance", "Angle"])
        for framenum,i,dx,dy,distance,angle in all_rel_metrics:
            t=(1/fps)*framenum
            writer.writerow([t,i,dx,dy,distance,angle])

    with open(f"{folder}/TailSegments_Relative_Metrics.csv", mode='w', newline='') as file:
        writer = csv.writer(file)
        #adds meta data to the top of the CSV
        writer.writerow(["Number of Boxes","Total Time","Start Frame","End Frame","Video Path","Alpha","Beta"])
        writer.writerow([num_boxes,(end_frame-start_frame)*(1/fps),start_frame,end_frame,filepath,alpha,beta])

        #adds real formatting for data
        writer.writerow(["Frame", "Box Index", "dx (i+1 rel. i)", "dy (i+1 rel. i)", "Distance (i+1 rel. i)", "Angle (i+1 rel. i)"])
        for framenum,i,dx,dy,distance,angle in all_rel_metrics_2:
            t=(1/fps)*framenum
            writer.writerow([t,i,dx,dy,distance,angle])

    times=[]
    angles=[]

    with open(f"{folder}/Total_Angle_Metrics.csv", mode='w', newline='') as file:
        writer = csv.writer(file)
        #adds meta data to the top of the CSV
        writer.writerow(["Number of Boxes","Total Time","Start Frame","End Frame","Video Path","Alpha","Beta"])
        writer.writerow([num_boxes,(end_frame-start_frame)*(1/fps),start_frame,end_frame,filepath,alpha,beta])

        #adds real formatting for data
        writer.writerow(["Time (s)", "Total Angle (deg)"])
        for framenum, total_angle_sum in all_abs_metrics:
            t=(1/fps)*framenum
            times.append(t)
            writer.writerow([times, total_angle_sum])
            angles.append(total_angle_sum)

    #need to have a FPS to time conversion (camera should operate at 30 fps, then I can measure the frame number 1/30 )
    # Save Position plot
    position_plot_path = os.path.join(folder, f"Time_vs_Frame_{formatted_datetime}.png")
    plt.figure()
    plt.plot(times, angles, label="Position")
    plt.xlabel("Time (s)")
    plt.ylabel("Angle Total")
    plt.title("Time vs. Angle")
    plt.grid(True)
    plt.savefig(position_plot_path)
    print(f"Position plot saved to {position_plot_path}")
    plt.pause(10)  # Keeps plot open for 10 seconds

    """
    The following graphs are all referencing frame vs angle, etc.
    # Save marker positions
    np.savetxt(f"{folder}/markers.csv", markers, delimiter=",")
    with open(f"{folder}/TailSegments_to_HeadSegments_Metrics.csv", mode='w', newline='') as file:
        writer = csv.writer(file)
        #adds meta data to the top of the CSV
        writer.writerow(["Number of Boxes","Total Frames","Start Frame","End Frame","Video Path","Alpha","Beta"])
        writer.writerow([num_boxes,end_frame-start_frame,start_frame,end_frame,filepath,alpha,beta])

        #adds real formatting for data
        writer.writerow(["Frame", "Box Index", "dx", "dy", "Distance", "Angle"])
        for entry in all_rel_metrics:
            writer.writerow(entry)

    #should format this better, then add a first row with the "meta" data and a second row with the cumulative data 
    with open(f"{folder}/TailSegments_Relative_Metrics.csv", mode='w', newline='') as file:
        writer = csv.writer(file)
        #adds meta data to the top of the CSV
        writer.writerow(["Number of Boxes","Total Frames","Start Frame","End Frame","Video Path","Alpha","Beta"])
        writer.writerow([num_boxes,end_frame-start_frame,start_frame,end_frame,filepath,alpha,beta])

        #adds real formatting for data
        writer.writerow(["Frame", "Box Index", "dx (i+1 rel. i)", "dy (i+1 rel. i)", "Distance (i+1 rel. i)", "Angle (i+1 rel. i)"])
        for entry in all_rel_metrics_2:
            writer.writerow(entry)

    #should format this better, then add a first row with the "meta" data and a second row with the cumulative data 
    #add a cumulative sum on the oustide of the for-loop to measure the total angle 
    frames=[]
    angles=[]
    with open(f"{folder}/Total_Angle_Metrics.csv", mode='w', newline='') as file:
        writer = csv.writer(file)
        #adds meta data to the top of the CSV
        writer.writerow(["Number of Boxes","Total Frames","Start Frame","End Frame","Video Path","Alpha","Beta"])
        writer.writerow([num_boxes,end_frame-start_frame,start_frame,end_frame,filepath,alpha,beta])

        for framenum, total_angle_sum in all_abs_metrics:
            writer.writerow([framenum, total_angle_sum])
            frames.append(framenum)
            angles.append(total_angle_sum)

    # Save Position plot
    position_plot_path = os.path.join(folder, f"Angle_vs_Frame_{formatted_datetime}.png")
    plt.figure()
    plt.plot(frames, angles, label="Position")
    plt.xlabel("Frames")
    plt.ylabel("Angle Total")
    plt.title("Frame vs. Angle")
    plt.grid(True)
    plt.savefig(position_plot_path)
    print(f"Position plot saved to {position_plot_path}")
    plt.pause(10)  # Keeps plot open for 10 seconds
    """

if __name__ == "__main__":
    track_markers()
