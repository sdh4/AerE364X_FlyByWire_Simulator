

class Direction(object):
    INPUT=0
    OUTPUT=1
    pass

DigitalInOuts_by_pin={}

class DigitalInOut(object):
    pin=None
    direction=None
    value=None
    
    def __init__(self,pin):
        self.pin=pin
        self.direction=Direction.INPUT
        self.value=False
        DigitalInOuts_by_pin[pin.function]=self
        
        pass
    
