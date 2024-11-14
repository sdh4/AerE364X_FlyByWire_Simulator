import math
import busio

import dynamic_model
import board


# Note that this module
# accesses the dynamic_model
# directly rather than getting
# written to via the pin mapping in
# board.py. Perhaps a cleaner
# more consistent architecture
# would be to have an I2C device
# mapping similar to the pin
# mapping in board.py


class BMP3XX_I2C(object):
    
    def __init__(self,busio_object,address = 0X77):
        if not isinstance(busio_object,busio.I2C):
            raise ValueError("Invalid I2C bus for BMP3XX")
        if not board.bmp3xx_i2c:
            raise ValueError("BMP3XX  not enabled in board.py")
        
        pass
    @property
    def altitude(self):
        with dynamic_model.dynamic_instance.model_lock:
        
            return dynamic_model.dynamic_instance.pos[2]
        pass
    

    @property
    def temperature(self):
        return 22.0
    pass

