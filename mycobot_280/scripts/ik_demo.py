#! /usr/bin/env python3.7
import torch
import torch.nn as nn
from torch import optim
import torch.nn.functional as F
from torch.utils.data import DataLoader
from torch.utils.data import Dataset
import numpy as np
import matplotlib.pyplot as plt
import os
import sys
import pandas as pd

args = sys.argv
print(len(args))
assert len(args)>=2, "you must specify the argument."

#gpu check
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(device)

#get path
filename=args[1]
print(filename)

#train data
# x = np.linspace(0, 4*np.pi)
# sin_x = np.sin(x) + np.random.normal(0, 0.3, len(x))
df_jointangles = pd.read_csv(filename)
np_jointangles = df_jointangles.to_numpy()
print(np_jointangles.shape)

#plot
def plot_data(input):
    fig, axes = plt.subplots(2, 3, figsize=(16,12))
    c = 0
    for i in range(2):
        for j in range(3):
            axes[i][j].plot(input[:,c], 'r', label="j"+str(c+1))
            axes[i][j].set_title("j"+str(c+1))
            axes[i][j].set_xlabel("time[s]")
            axes[i][j].set_ylabel("angle[rad]")
            c += 1
    plt.show()
data = np_jointangles

# hyperparameter
nb_epoches = 10000
batch_size = 64
seed_val = 1
model_path = 'model.pth'

# Model
class IKModel(nn.Module):
    def __init__(self, input_size:int, hidden_size:list, output_size:int):
        super().__init__()
        self.fc1 = nn.Linear(input_size,  hidden_size[0])
        # self.bn1 = nn.BatchNorm1d(hidden_size[0])
        self.fc2 = nn.Linear(hidden_size[0], hidden_size[1])
        # self.bn2 = nn.BatchNorm1d(hidden_size[1])
        self.fc3 = nn.Linear(hidden_size[1], hidden_size[2])
        # self.bn3 = nn.BatchNorm1d(hidden_size[2])
        self.fc4 = nn.Linear(hidden_size[2], hidden_size[3])
        # self.bn4 = nn.BatchNorm1d(hidden_size[3])
        self.fc5 = nn.Linear(hidden_size[3], output_size)      
        self.relu = nn.ReLU()
                
    def forward(self, x):
        x = self.relu(self.fc1(x))
        x = self.relu(self.fc2(x))
        x = self.relu(self.fc3(x))
        x = self.relu(self.fc4(x))
        x = self.fc5(x)
        return x
    
# dataset
class CustomDataset(Dataset):
    def __init__(self, x_data, y_data):
        self.x = x_data
        self.y = y_data

    def __len__(self):
        return len(self.x)
    
    def __getitem__(self, idx):
        #data to tensor
        x = torch.FloatTensor(self.x[idx])
        y = torch.FloatTensor(self.y[idx])
        return x, y

    
model = IKModel(input_size=6, hidden_size=[256, 128, 64, 32], output_size=6).to(device)
#cost
cost_func = nn.MSELoss().to(device)
#optim
optimizer = optim.SGD(model.parameters(), lr=0.01)
# optimizer = optim.Adam(model.parameters())

#dataload
x_data = data[:-1,:]
y_data = data[1:,:]

# tensor to dateset
train_dataset = CustomDataset(x_data, y_data)
# dataset to dataloader
train_loader = DataLoader(train_dataset, batch_size = batch_size)

a, b = next(iter(train_loader))
print(a.shape)

#for record
record_loss_train = []

# train
for epoch in range(1, nb_epoches+1):
    avg_cost = 0
    total_batch = len(train_loader)
    min_cost = 180

    for x_train, y_train in train_loader:
        x_train = x_train.to(device)
        y_train = y_train.to(device)
        y_pred = model(x_train)
        
        optimizer.zero_grad()
        cost = cost_func(y_pred, y_train)
        cost.backward()
        optimizer.step()
        avg_cost += cost / total_batch
    if epoch % 10 == 0: 
        print("Epoch:{}/{} Cost:{:.6f}".format(epoch, nb_epoches, avg_cost))             
    if min_cost > avg_cost:
        torch.save(model.state_dict(), model_path)
    
        
# test
model = IKModel(input_size=6, hidden_size=[256, 128, 64, 32], output_size=6).to(device)
model.load_state_dict(torch.load(model_path))
test_result = []
model.eval()
x_test = torch.FloatTensor(x_data[0]).to(device)
print("x_test:",x_test)
with torch.no_grad():
    for i in range(213):
        y_pred = model(x_test).to(device)
        x_test = y_pred
        np_pred = y_pred.to("cpu").detach().numpy().copy()
        test_result.append(np_pred)

#testplot
test_result = np.array(test_result)
print(test_result.shape)
plot_data(test_result)

