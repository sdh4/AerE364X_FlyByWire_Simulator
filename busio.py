class I2C(object):

    def __init__(self,clock_pin,data_pin):
        self.clock_pin=clock_pin
        self.data_pin=data_pin
    
        if clock_pin.function != "SCL":
            raise ValueError("Invalid I2C clock pin")
        
        if data_pin.function != "SDA":
            raise ValueError("Invalid I2C data pin")
        pass
    
    pass
