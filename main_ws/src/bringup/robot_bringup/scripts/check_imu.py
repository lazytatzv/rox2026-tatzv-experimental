#!/usr/bin/env python3
import os
import fcntl
import sys

I2C_SLAVE = 0x0703

def check_imu(bus="/dev/i2c-1", addr=0x28):
    if not os.path.exists(bus):
        return False
    try:
        fd = os.open(bus, os.O_RDWR)
        try:
            fcntl.ioctl(fd, I2C_SLAVE, addr)
            # Try to read one byte (Chip ID register 0x00)
            os.write(fd, b'\x00')
            os.read(fd, 1)
            return True
        except OSError:
            return False
        finally:
            os.close(fd)
    except OSError:
        return False

if __name__ == '__main__':
    if check_imu():
        print("true", end="")
    else:
        print("false", end="")
