import cv2
from datetime import datetime
#streams usb-camera video to my computer 

# Get the current timestamp for the start time
start_time = datetime.now().strftime("%m-%d_%H-%M-%S")

# Open the USB camera -1 for an external camear 
camera = cv2.VideoCapture(1)

# Check if the camera opened successfully
if not camera.isOpened():
    print("Error: Could not open camera.")
    exit()

# Define the codec and create a VideoWriter object
# 'XVID' is a common codec; you can use others like 'MJPG' or 'MP4V'
fourcc = cv2.VideoWriter_fourcc(*'MP4V')
out = cv2.VideoWriter(f'output_{start_time}.avi', fourcc, 20.0, (640, 480))

print("Recording... Press 'q' to stop.")

while True:
    ret, frame = camera.read()
    if not ret:
        print("Error: Failed to capture frame.")
        break

    # Add title to the frame
    cv2.putText(frame, f"START TIME: {start_time}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)

    # Write the frame to the output file
    out.write(frame)

    # Display the frame (optional)
    cv2.imshow('Recording', frame)

    # Stop recording when 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        end_time = datetime.now().strftime("%m-%d_%H-%M-%S")
        break

# Release resources
camera.release()
out.release()
cv2.destroyAllWindows()
