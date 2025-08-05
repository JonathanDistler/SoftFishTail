import torch
from torch import nn

# Define the model class exactly as it was during training
class NeuralNetwork(nn.Module):
    def __init__(self):
        super().__init__()
        self.flatten = nn.Flatten()
        self.linear_relu_stack = nn.Sequential(
            nn.Linear(28*28, 512),
            nn.ReLU(),
            nn.Linear(512, 512),
            nn.ReLU(),
            nn.Linear(512, 10)
        )

    def forward(self, x):
        x = self.flatten(x)
        logits = self.linear_relu_stack(x)
        return logits

# Recreate the model structure and load weights
model = NeuralNetwork()
model.load_state_dict(torch.load("model.pth", weights_only=True))  # or use full path
model.eval()

print("Model loaded and ready to use.")


#now for making a prediction using the previously trained model
from PIL import Image
from torchvision import transforms
import numpy as np
import cv2 as cv


#examples of previously calibrated camera matrcies. These were done at a much longer distance than a viable rate, and out of water, so not useful for real testing
camera_matrix = np.array([
    [6.85402255e+03, 0.00000000e+00, 9.53349510e+02],
    [0.00000000e+00, 6.32972942e+03, 5.34921515e+02],
    [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
])

dist_coeffs = np.array([[1.93965100e+01, 1.03302061e+03, 6.03419410e-01, -6.17968976e-02, -1.74216458e-01]])

def undistort_image(image, camera_matrix, dist_coeffs, interpolation=cv.INTER_LINEAR):
    h, w = image.shape[:2]
    # Calculate the new optimal camera matrix (optional but usually done)
    newcameramtx, _ = cv.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (w, h), 1, (w, h))
    
    # Initialize undistort rectify map
    mapx, mapy = cv.initUndistortRectifyMap(camera_matrix, dist_coeffs, None, newcameramtx, (w, h), cv.CV_32FC1)

    # Remap the image using the computed maps
    newimage = cv.remap(image, mapx, mapy, interpolation)
    return newimage


# Load image, or this would be the image taken by the fish underwater. Then, one would apply my undistortion algorithm/function to undistort the image
#This would be an example of an outputted and undistorted image. If it had been taken underwater, it would need to be undistorted with previous function undistort_image
name="/home/srl-slim-tim/Pictures/Screenshots/MNIST-1.png"

img = Image.open(name).convert("L")  # convert to grayscale

transform = transforms.Compose([
    transforms.Resize((28, 28)),
    transforms.ToTensor(),  # shape: [1, 28, 28]
])

tensor_img = transform(img)  # shape: [1, 28, 28]
tensor_img = tensor_img.view(1, -1)  # flatten to [1, 784]

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
model.to(device)
input_tensor = tensor_img.to(device)

with torch.no_grad():
    output = model(input_tensor)  # shape depends on model

# For classification, get predicted class
_, predicted_class = torch.max(output, 1)
print(f"Predicted class index (0-9): {predicted_class.item()}")
index_val=name.find("MNIST-")
index_val=index_val+6
print("Real value:",name[index_val:index_val+1])
