import math
import busio

import dynamic_model


# Note that this module
# accesses the dynamic_model
# directly rather than getting
# written to via the pin mapping in
# board.py. Perhaps a cleaner
# more consistent architecture
# would be to have an I2C device
# mapping similar to the pin
# mapping in board.py


class BNO055_I2C(object):

    def __init__(self,busio_object):
        if not isinstance(busio_object,busio.I2C):
            raise ValueError("Invalid I2C bus for BNO055")

        
        pass
    @property
    def calibrated(self):
        return dynamic_model.dynamic_instance.imu_calibrated

    @property
    def quaternion(self):
        return dynamic_model.dynamic_instance.quat

    @property
    def euler(self):
        # returns Euler angles in degrees... NOTE
        # Range of roll/pitch values may be incorrect for the BNO055
        imu_euler_vals = dynamic_model.dynamic_instance.imu_euler*180/math.pi
        imu_euler_vals[0] = -imu_euler_vals[0]  # negate heading to go from positive ccw for z axis coming out of the ground, to positive cw for compass heading
        return imu_euler_vals

    @property
    def gyro(self):
        return dynamic_model.dynamic_instance.imu_gyro  # returns in rad/sec
    
    pass
