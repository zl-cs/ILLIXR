from tracemalloc import start
import numpy as np
import matplotlib.pyplot as plt
import csv

receive_time = {}
vio_time = {}
pose_transfer_time = {}
roundtrip_time = {}

DIR_NAME = "recorded_data"

with open('../' + DIR_NAME + '/receive_time.csv', mode='r') as input:
    reader = csv.reader(input)
    receive_time = {int(rows[0]): (np.int64(rows[1]), float(rows[2])) for rows in reader}
    # print(receive_time)

with open('../' + DIR_NAME + '/receiver_to_sender_time.csv', mode='r') as input:
    reader = csv.reader(input)
    vio_time = {int(rows[0]): (np.int64(rows[1]), float(rows[2])) for rows in reader}

with open('../' + DIR_NAME + '/pose_transfer_time.csv', mode='r') as input:
    reader = csv.reader(input)
    pose_transfer_time = {int(rows[0]): (np.int64(rows[1]), float(rows[2])) for rows in reader}

with open('../' + DIR_NAME + '/roundtrip_time.csv', mode='r') as input:
    reader = csv.reader(input)
    roundtrip_time = {int(rows[0]): (np.int64(rows[1]), float(rows[2])) for rows in reader}

min_idx = int(min(max(receive_time.keys()), max(vio_time.keys()), max(roundtrip_time.keys())))
receive_time = {k: v for k, v in receive_time.items() if int(k) <= int(min_idx)}
vio_time = {k: v for k, v in vio_time.items() if int(k) <= int(min_idx)}
pose_transfer_time = {k: v for k, v in pose_transfer_time.items() if int(k) <= int(min_idx)}
roundtrip_time = {k: v for k, v in roundtrip_time.items() if int(k) <= int(min_idx)}

# Create data
x_range = np.arange(min_idx)
y1 = [receive_time[i][1] if i in receive_time else 0 for i in x_range]
y2 = [vio_time[i][1] if i in vio_time else 0 for i in x_range]
# y3 = [pose_transfer_time[i][1] if i in pose_transfer_time else 0 for i in x_range]
y4 = [(roundtrip_time[i][1] - (y1[i] + y2[i])) if i in roundtrip_time else 0 for i in x_range]



end_time = roundtrip_time[max(roundtrip_time.keys())][0]
start_time = roundtrip_time[min(roundtrip_time.keys())][0]

duration = (end_time - start_time) / 1e9
x_range = x_range / min_idx  * duration
print(duration)

x_ticks = np.arange(0, duration, 2)

# Basic stacked area chart.
plt.stackplot(x_range, y1, y2, y4, labels=['Camera + IMU Transfer Time', 'Server Time', 'Pose Transfer Time'])
plt.legend(loc='upper left')
plt.xticks(x_ticks)

mean = np.mean([y for (x, y) in roundtrip_time.values()])
plt.axhline(y=mean, color='r', linestyle='-')
# x1,x2,y1,y2 = plt.axis()
# plt.axis((x1, x2, 0, 175))

plt.show()