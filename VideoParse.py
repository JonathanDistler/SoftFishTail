import cv2
import numpy as np
import time
#This allows you to parse through a video frame by frame and edit it with the camera calibration. This could be used well for a post-process script. 

# Input and output paths
video_path = "C:/Users/15405/OneDrive/Desktop/Career/ETHZ/ETHZ Work/HardwareOutput/output_07-14_11-03-34.mp4"
output_path = "C:/Users/15405/OneDrive/Desktop/Career/ETHZ/ETHZ Work/HardwareOutput/undistorted_output.avi"

# Load video
video_capture = cv2.VideoCapture(video_path)

# Camera calibration
mtx = np.array([[794.06933744, 0., 931.41508085],
                [0., 794.10552238, 539.85862057],
                [0., 0., 1.]])
dist = np.array([-2.75729559e-01,  1.04389635e-01, -4.49111933e-04, -7.43910505e-05, -2.53113830e-02])

# Check video loaded
if not video_capture.isOpened():
    print("Error: Could not open video file.")
    exit()

fps = video_capture.get(cv2.CAP_PROP_FPS)
scale = 2  # Scaling factor

out_video = None
frame_number = 0

while True:
    ret, frame = video_capture.read()
    if not ret:
        print("End of video reached.")
        break

    frame_number += 1
    print(f"Processing frame {frame_number}")
    
    h, w = frame.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
    dst = cv2.undistort(frame, mtx, dist, None, newcameramtx)
    x, y, w_roi, h_roi = roi
    dst = dst[y:y+h_roi, x:x+w_roi]

    resized = cv2.resize(dst, None, fx=scale, fy=scale)

    # Initialize video writer once frame size is known
    if out_video is None:
        height, width = resized.shape[:2]
        output_size = (width, height)
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        out_video = cv2.VideoWriter(output_path, fourcc, fps, output_size)
        if not out_video.isOpened():
            print("Error: Could not open video writer.")
            video_capture.release()
            exit()

    # Double-check output size matches
    if (resized.shape[1], resized.shape[0]) != output_size:
        print(f"Size mismatch, skipping frame {frame_number}")
        continue

    out_video.write(resized)
    cv2.imshow("Frame", resized)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Clean up
video_capture.release()
if out_video:
    out_video.release()
cv2.destroyAllWindows()
