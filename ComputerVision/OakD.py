import depthai as dai
import cv2

#Oak-D camera
#has maximum FPS up to 60
#code to run the OakD camera

# Create a pipeline
pipeline = dai.Pipeline()

# Define a color camera node
cam_rgb = pipeline.createColorCamera()
cam_rgb.setPreviewSize(1920, 1080)
cam_rgb.setInterleaved(False)
cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

# Create an XLink output node
xout = pipeline.createXLinkOut()
xout.setStreamName("video")
cam_rgb.preview.link(xout.input)

# Connect to device and start pipeline
with dai.Device(pipeline) as device:
    video_queue = device.getOutputQueue(name="video", maxSize=4, blocking=False)

    while True:
        in_frame = video_queue.get()
        frame = in_frame.getCvFrame()

        cv2.imshow("OAK-D RGB Preview", frame)

        if cv2.waitKey(1) == ord('q'):
            break

cv2.destroyAllWindows()
