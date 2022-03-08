PWMOuts_by_pin={}  # Dictionary of PWMOut objects 

PWMNotifiers_by_pin={}  # Dictionary of lists

class PWMOut(object):
    pin=None
    duty_cyc=None
    freq=None
    variable_frequency=None

    def __init__(self,pin,duty_cycle=0,frequency=500,variable_frequency=False):
        self.pin=pin
        self.duty_cyc=duty_cycle
        self.freq=frequency
        self.variable_frequency=variable_frequency
        PWMOuts_by_pin[pin.function]=self # register in the global dictionary

        self._notify()
        pass


    @property
    def frequency(self):
        return self.freq

    @frequency.setter
    def frequency(self,value):
        if not self.variable_frequency:
            raise ValueError("Can not set frequency unless variable_frequency is True")
        self.freq=value
        pass

    @property
    def duty_cycle(self):
        return self.duty_cyc

    @duty_cycle.setter
    def duty_cycle(self,value):
        self.duty_cyc=value
        self._notify()
        pass


    def _notify(self):
        if self.pin.function in PWMNotifiers_by_pin:
            NotifierList = PWMNotifiers_by_pin[self.pin.function]
            for notifier in NotifierList:
                notifier()
                pass
            pass
        pass
    
    pass
