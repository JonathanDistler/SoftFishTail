import torch
from torch import nn

# Define the model class exactly as it was during training, this was hardcoded. future iterations could process the parameters and input as variables
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


#PREDICTION USING MODEL#
from PIL import Image
from torchvision import transforms


# Load image
#In the autonomous movements for the SoFi fish, this would be a picture taken underwater, then called with the previously defined undistort function for a user-defined (and tested distance, ie 10 cm). This is an example usecase:
name="SoftFishTail/ML/MNIST-1.png"

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
