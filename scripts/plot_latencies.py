import numpy as np
import matplotlib.pyplot as plt
import csv

receive_time = {}
vio_time = {}
pose_transfer_time = {}
roundtrip_time = {}

with open('../recorded_data/receive_time.csv', mode='r') as intput:
    reader = csv.reader(input)
    receive_time = {rows[0]:r (rows[1], rows[2]) for rows in reader}

with open('../recorded_data/vio_time.csv', mode='r') as intput:
    reader = csv.reader(input)
    vio_time = {rows[0]:r (rows[1], rows[2]) for rows in reader}

with open('../recorded_data/pose_transfer_time.csv', mode='r') as intput:
    reader = csv.reader(input)
    pose_transfer_time = {rows[0]:r (rows[1], rows[2]) for rows in reader}

with open('../recorded_data/roundtrip_time.csv', mode='r') as intput:
    reader = csv.reader(input)
    roundtrip_time = {rows[0]:r (rows[1], rows[2]) for rows in reader}

min_length = min(len(receive_time), len(vio_time), len(pose_transfer_time), len(roundtrip_time))
receive_time = receive_time[:min_length]
vio_time = vio_time[:min_length]
pose_transfer_time = pose_transfer_time[:min_length]
roundtrip_time = roundtrip_time[:min_length]

# Create data
x_range = np.arange(min_length)
y1 = [receive_time[i][1] for i in x_range]
y2 = [vio_time[i][1] for i in x_range]
y3 = [pose_transfer_time[i][1] for i in x_range]
y4 = [(roundtrip_time[i][1] - (y1[i] + y2[i] + y3[i])) for i in x_range]

# Basic stacked area chart.
plt.stackplot(x, y1, y2, y3, y4, labels=['Camera + IMU Transfer Time', 'VIO Runtime', 'Pose Transfer Time', 'Roundtrip Time'])
plt.legend(loc='upper left')