from serial.tools import list_ports
from serial import Serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
import numpy as np


# Find port and connect
for port, desc, hwid in list_ports.comports():
    if "STLink" in desc:
        ct_port = Serial(port, 230400, timeout=None)
        break

assert ct_port is not None, "No STLink found"

class IMU:
    def __init__(self, ts: int, mag, acc, gyr):
        self.ts = ts
        self.mag = mag
        self.acc = acc
        self.gyr = gyr
        self.line = [self.ts]
        self.line.extend(self.mag)
        self.line.extend(self.acc)
        self.line.extend(self.gyr)
        self.line = np.array(self.line, dtype=np.float64)

    def mag_norm(self) -> float:
        return np.linalg.norm(self.mag)

    def __str__(self) -> str:
        return f"ts: {self.ts}\tmag: {self.mag}\taccel: {self.acc}\tgyro: {self.gyr}"
    
def str_to_imu(data: bytes) -> IMU:
    data = data.decode("ascii")
    data = data.split("\t")

    ts = data[0]
    mag = data[1]
    acc = data[2]
    gyr = data[3]

    ts = int(ts.removeprefix("ts: "))
    mag = mag.removeprefix("mag: ").split(" ")
    acc = acc.removeprefix("accel: ").split(" ")
    gyr = gyr.removeprefix("gyro: ").split(" ")

    mag = [float(mag[0]), float(mag[1]), float(mag[2])]
    acc = [float(acc[0]), float(acc[1]), float(acc[2])]
    gyr = [float(gyr[0]), float(gyr[1]), float(gyr[2])]
    return IMU(ts, mag, acc, gyr)


style.use("fivethirtyeight")

fig = plt.figure()
ax1 = fig.add_subplot(1, 1, 1)
data: np.array = np.empty((1,10))


def animate(i, data):
    while True:
        try:
            imu = str_to_imu(ct_port.read_until(b"\n"))
            data = data.append([imu.line], axis=0)
            break
        except (IndexError, ValueError):
            pass
    ax1.clear()
    ax1.plot(data[:,0], data[:,1])

ani = animation.FuncAnimation(fig, animate, fargs=(data), interval=10)

plt.show()