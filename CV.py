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
def track_markers(
    filepath: str, num_boxes: int, start_frame: int, end_frame: int
):
    """Track bounding boxes within video.
    Script for tracking N manually chosen bounding boxes within video. Point this script to the video file you would like to track and choose how many N bounding boxes are desired. These bounding box centers (markers) are stored in a CSV file afterwards. Two videos are created as well, one is the original video cut to [start_frame, end_frame], and the other is with the tracking bounding boxes displayed

    Args:
        filepath (str): Video filepath.
        num_boxes (int): Number of bounding boxes.
        start_frame (int): Starting frame of video at which tracking should start.
        end_frame (int): Last frame of video that should be considered (inclusive) during tracking.
    """
    #video_1=r"C:\Users\15405\OneDrive\Desktop\Career\ETHZ\ETHZ Work\20250704_140041.mp4"
    #video_2=r"C:\Users\15405\OneDrive\Desktop\Career\ETHZ\ETHZ Work\20250704_140610.mp4"
    #could work on hardcoding filepathinto code
    folder = os.path.dirname(filepath)
    cap = cv2.VideoCapture(filepath)

    # Format the date and time
    # Generate a timestamp without invalid characters
    formatted_datetime = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")


    framenum = 0
    angles=[]
    while cap.isOpened():
        ret, frame = cap.read()
        framenum += 1
        print(framenum)
        if framenum < start_frame:
            continue

        height, width = frame.shape[:2]
        y_center = height // 2  # vertical center (y-coordinate)
        x_center=width//2 #horizontal center (x-coordinate)

        start_point = (0, y_center)  # left edge of screen
        center_point = (width // 2, y_center)  # horizontal center of screen
        end_point = (width - 1, y_center)  # right edge
        #need a mount to hold the camera still, so the start position is maintained throughout the whole video
        bboxes = []
        for i in range(num_boxes):
            bboxes.append(cv2.selectROI(f"Select {i+1}-th Marker", frame))
        break
    else:
        print("ERROR loading file")
    cap.release()
    cv2.destroyAllWindows()

    # Create multiple trackers
    trackers = cv2.legacy.MultiTracker_create()
    for i in range(num_boxes):
        trackers.add(cv2.legacy.TrackerCSRT_create(), frame, bboxes[i])

    # Tracking
    cap = cv2.VideoCapture(filepath)
    fps = cap.get(cv2.CAP_PROP_FPS)

    # Store keypoint tracked video, using MP4 format.
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    cut_movie = cv2.VideoWriter(
        f"{folder}/cut_video.mp4",
        fourcc,
        fps,
        (frame.shape[1], frame.shape[0]),
    )
    tracked_movie = cv2.VideoWriter(
        f"{folder}/tracked_video_{formatted_datetime}.mp4",
        fourcc,
        fps,
        (frame.shape[1], frame.shape[0]),
    )


    markers = []
    framenum = 0
    while cap.isOpened():
        ret, frame = cap.read()
        #higher brightness has been giving a better binding box
        #higher contrast was poor for bounding box accuracy
        frame=adjust_contrast(frame,1,75)
        framenum += 1
        print(framenum)


        # Draw white line, visual reference for center of the screen
        cv2.line(frame, start_point, end_point, color=(255, 255, 255), thickness=1)
        if framenum < start_frame:
            continue
        if not ret or framenum > end_frame:
            break

        cut_movie.write(frame)
        # Give tracker new frame with minimal movement
        found, bboxes_new = trackers.update(frame)

        if not found:
            cv2.putText(
                frame,
                text="One of the objects not found",
                org=(20, 70),
                fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                fontScale=0.75,
                color=(0, 0, 255),
                thickness=2,
            )

        for box in bboxes_new:
            topleft = (int(box[0]), int(box[1]))
            botright = (int(box[0] + box[2]), int(box[1] + box[3]))

            center_x_box = int(box[0] + box[2] / 2)
            center_y_box = int(box[1] + box[3] / 2)
            #draws rectangle based on corners
            cv2.rectangle(
                frame, topleft, botright, color=(0, 0, 255), thickness=2
            )

            pts = np.array([[x_center, y_center], [center_x_box, center_y_box]], np.int32)
            pts = pts.reshape((-1, 1, 2))  # Reshape to (n_points, 1, 2)
            cv2.polylines(frame, [pts], isClosed=False, color=(0, 255, 255), thickness=2)

            #computes math based on center point of the rectangle
            delta_y=y_center-center_y_box #delta height from center rectangle to center frame
            delta_x=center_x_box-x_center #delta width from center rectangle to center of frame
            angle=math.atan(delta_y/delta_x)
            angle=angle*180
            angle=angle/3.1415 #converts to degrees
            print(angle)
            angles.append(angle)

        curr_markers = []
        # Compute relevant values of motion marker positions
        for i in range(num_boxes):
            # Compute centers of found bounding boxes
            curr_center = np.array(
                [
                    bboxes_new[i][0] + bboxes_new[i][2] / 2,
                    bboxes_new[i][1] + bboxes_new[i][3] / 2,
                ]
            )
            curr_markers.append(curr_center)

        # Markers are stored as a flattened array, x then y coordinate of each point.
        markers.append(np.array(curr_markers).flatten())

        cv2.imshow("Tracker (press Q to exit early)", frame)
        tracked_movie.write(frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            print("Exited early!")
            break

    cut_movie.release()
    tracked_movie.release()
    cap.release()
    cv2.destroyAllWindows()

    np.savetxt(f"{folder}/markers.csv", markers, delimiter=",")
    # Save angles with frame numbers
    with open(f"{folder}/angles.csv", mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Frame", "Angle (degrees)"])
        for i, angle in enumerate(angles, start=start_frame):
            writer.writerow([i, angle])
if __name__ == "__main__":
    track_markers()


#notes
#could add a bounding box that follows the moving frame
#could also use a different AI model that is better with movements
#or. . . could slow down the video by 1/2 first, then double teh number of frames, so that it could treat it better 
#could have higher fps to have better tracking
