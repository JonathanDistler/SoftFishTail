import torch
from torch import nn
from torch.utils.data import DataLoader
from torchvision import datasets
from torchvision.transforms import ToTensor
import random

#download torch with my conda environment, then run this script on the conda enviornment. This worked very quickly on the Ubuntu lab laptop of the SRL, however, much slower on my personal HP device
#almost all of the code is from https://docs.pytorch.org/tutorials/beginner/basics/quickstart_tutorial.html, I added the random guess at the end, as well as 6 epochs instead of 5. It might be overtaining, will need to look into it more

# Download training data from open datasets, this is a dataset of handdrawn numbers, class ranges from integers of 0 to 9
training_data = datasets.MNIST(
    root="data",
    train=True,
    download=True,
    transform=ToTensor(),
)

# Download test data from open datasets.
test_data = datasets.MNIST(
    root="data",
    train=False,
    download=True,
    transform=ToTensor(),
)

batch_size = 64

# Create data loaders.
train_dataloader = DataLoader(training_data, batch_size=batch_size)
test_dataloader = DataLoader(test_data, batch_size=batch_size)

for X, y in test_dataloader:
    print(f"Shape of X [N, C, H, W]: {X.shape}")
    print(f"Shape of y: {y.shape} {y.dtype}")
    break

#lab computer had Geforce RTX GPU, my normal laptop just had cpu capabilities 
device = "cuda" if torch.cuda.is_available() else "cpu"
print(f"Using {device} device")

# Define model
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

model = NeuralNetwork().to(device)
print(model)

loss_fn = nn.CrossEntropyLoss()
optimizer = torch.optim.SGD(model.parameters(), lr=1e-3)

def train(dataloader, model, loss_fn, optimizer):
    size = len(dataloader.dataset)
    model.train()
    for batch, (X, y) in enumerate(dataloader):
        X, y = X.to(device), y.to(device)

        # Compute prediction error
        pred = model(X)
        loss = loss_fn(pred, y)

        # Backpropagation
        loss.backward()
        optimizer.step()
        optimizer.zero_grad()

        if batch % 100 == 0:
            loss, current = loss.item(), (batch + 1) * len(X)
            print(f"loss: {loss:>7f}  [{current:>5d}/{size:>5d}]")

def test(dataloader, model, loss_fn):
    size = len(dataloader.dataset)
    num_batches = len(dataloader)
    model.eval()
    test_loss, correct = 0, 0
    with torch.no_grad():
        for X, y in dataloader:
            X, y = X.to(device), y.to(device)
            pred = model(X)
            test_loss += loss_fn(pred, y).item()
            correct += (pred.argmax(1) == y).type(torch.float).sum().item()
    test_loss /= num_batches
    correct /= size
    print(f"Test Error: \n Accuracy: {(100*correct):>0.1f}%, Avg loss: {test_loss:>8f} \n")


#alternatively, could try more epochs, but that runs the risk of overfitting the data, it had previously been 5, but with only ~75% accuracy. 
epochs = 6
for t in range(epochs):
    print(f"Epoch {t+1}\n-------------------------------")
    train(train_dataloader, model, loss_fn, optimizer)
    test(test_dataloader, model, loss_fn)
print("Done!")
"""
torch.save(model.state_dict(), "model.pth")
print("Saved PyTorch Model State to model.pth")
"""

#saves model to the same folder this ML file is located
import os
model_path = os.path.abspath("model.pth")
print(f"Saved PyTorch Model State to: {model_path}")

#class is defined as handwritten numbers from 0 to 9
classes = [
    "0",
    "1",
    "2",
    "3",
    "4",
    "5",
    "6",
    "7",
    "8",
    "9",
]

#evaluates the model on a single randomly generated test sample 
model.eval()

rand_guess=random.randint(0,len(test_data)-1)
x, y = test_data[rand_guess][0], test_data[rand_guess][1]
with torch.no_grad():
    x = x.to(device)

    #uses model to predict which class value is generated (from a random index)
    pred = model(x) 
    #guesses a random class value to compare with prediction
    guess=random.choice(classes)

    predicted, actual = classes[pred[0].argmax(0)], classes[y]
    print(f'Predicted: "{predicted}", Actual: "{actual}, Random Guess {guess}"')


#do the same type of thing but with hand drawn number class and random guess at the end plus save to a folder in my vscode folder on github
