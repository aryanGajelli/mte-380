from struct import pack, unpack

recv = 0x0001E0FE9040200299FDADFDDA
gyro = []
for i in range(3):
    raw = (recv >> (i*16)) & 0xFFFF
    signed = unpack('=h', pack('=H', raw & 0xffff))[0]
    data = signed / 131
    gyro.append(data)
print(gyro)

#  got accel: -0.001 4.753 17.129      gyro: 62.550 -199.351 -160.267
#  -160.267*131 = -20999.477