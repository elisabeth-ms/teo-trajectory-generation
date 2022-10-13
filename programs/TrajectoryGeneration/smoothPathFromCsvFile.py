
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import csv
from scipy import interpolate
import os

x = []
y = []
z = []
frames = []
qx = []
qy = []
qz = []
qw = []
with open('trajectories/test-right-arm-motion.csv') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    line_count = 0
    for row in csv_reader:
        frames.append(float(row[0]))
        x.append(float(row[1]))  
        y.append(float(row[2]))
        z.append(float(row[3]))
        qw.append(float(row[7]))
        qx.append(float(row[4]))
        qy.append(float(row[5]))
        qz.append(float(row[6]))
        line_count += 1

    print(f'Processed {line_count} lines.')
x = np.array(x)
frames = np.array(frames)
y = np.array(y)
z = np.array(z)
print(x)
fig, ax = plt.subplots(2)
fig.suptitle("Right arm motion.")

ax[0].plot(frames,x, label='x')
ax[0].plot(frames,y, label='y')
ax[0].plot(frames,z, label='z')
ax[0].legend()

ax[1].plot(frames, qx, label='qx')
ax[1].plot(frames, qy, label='qy')
ax[1].plot(frames, qz, label='qz')
ax[1].plot(frames, qw, label='qw')
ax[1].legend()
ax[0].grid(True)
ax[1].grid(True)
plt.show()
fig.savefig("trajectories/test-right-arm-motion.png")


# Smoothed
tck, u = interpolate.splprep([frames, x], s=0.02)
#create interpolated lists of points
frameXnew, xnew = interpolate.splev(u,tck)
tck, u = interpolate.splprep([frames, y], s=0.02)
#create interpolated lists of points
frameYnew, ynew = interpolate.splev(u,tck)

tck, u = interpolate.splprep([frames, z], s=0.02)
#create interpolated lists of points
frameZnew, znew = interpolate.splev(u,tck)

tck, u = interpolate.splprep([frames, qx], s=0.02)
#create interpolated lists of points
frameqxnew, qxnew = interpolate.splev(u,tck)

tck, u = interpolate.splprep([frames, qy], s=0.02)
#create interpolated lists of points
frameqynew, qynew = interpolate.splev(u,tck)

tck, u = interpolate.splprep([frames, qz], s=0.02)
#create interpolated lists of points
frameqznew, qznew = interpolate.splev(u,tck)

tck, u = interpolate.splprep([frames, qw], s=0.02)
#create interpolated lists of points
frameqwnew, qwnew = interpolate.splev(u,tck)

fig2,ax2 = plt.subplots(2)
fig2.suptitle('Smoothed right arm motion.')

ax2[0].plot(frameXnew, xnew, label="x")
ax2[0].plot(frameYnew, ynew, label='y')
ax2[0].plot(frameZnew, znew, label='z')
ax2[0].legend()

ax2[1].plot(frameqxnew, qxnew, label='qx')
ax2[1].plot(frameqynew, qynew, label='qy')
ax2[1].plot(frameqznew, qznew, label='qz')
ax2[1].plot(frameqwnew, qwnew, label='qw')
ax2[1].legend()


ax2[0].grid(True)
ax2[1].grid(True)
plt.show()
fig2.savefig("trajectories/test-right-arm-motion-smooth.png")

csv_file_write = open("trajectories/test-right-arm-motion-smooth.csv", "w")
writer = csv.writer(csv_file_write, dialect='excel')
for i in range(0, len(frameXnew)):
    rowData = [i, xnew[i], ynew[i], znew[i], qxnew[i], qynew[i], qznew[i], qwnew[i]]
    writer.writerow(rowData)
csv_file_write.close()
print(frameXnew.shape, frameYnew.shape, frameZnew.shape, frameqxnew.shape, frameqynew.shape, frameqznew.shape, frameqwnew.shape)