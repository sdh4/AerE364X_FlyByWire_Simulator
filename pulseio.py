import threading

PulseIns_by_pin={}


class PulseIn(object):
    pin=None
    maxlen=None
    idle_state=None
    curpulses=None
    paused=None
    ignore_idle_pulse=None

    PulseIn_Lock = None # Must hold this lock to modify curpulses list
    def __init__(self,pin,maxlen=2,idle_state=False):
        self.pin=pin
        self.maxlen=maxlen
        self.idle_state=idle_state
        self.curpulses=[]
        self.paused=False
        self.ignore_idle_pulse=True
        assert(idle_state==False)

        self.PulseIn_Lock = threading.Lock()
        
        PulseIns_by_pin[pin.function]=self # register in the global dictionary
        
        pass

    def __len__(self):
        return len(self.curpulses)
    
    def popleft(self):
        with self.PulseIn_Lock:
            retval=self.curpulses[0]
            self.curpulses=self.curpulses[1:]
            pass
        
        return retval

    def clear(self):
        with self.PulseIn_Lock:
            self.curpulses=[]
            pass
        pass
    
    
    def pause(self):
        self.paused=True
        pass

    def resume(self,trigger_duration=0):
        if trigger_duration != 0:
            raise ValueError("Invalid to generate a trigger on an input pin!")
        self.ignore_idle_pulse=True
        self.paused=False
        pass
    
    def __getitem__(self,index):
        return self.curpulses[index]
    
    def _Add_pulse(self,micros,sense): # sense is True for pos-going, False for neg-going
        if self.paused:
            # ignore new pulses if paused
            return
        
        with self.PulseIn_Lock:
            if self.ignore_idle_pulse and sense==self.idle_state:
                return
            if self.ignore_idle_pulse:
                self.ignore_idle_pulse = False
                
            self.curpulses.append(int(micros))
            if len(self.curpulses) > self.maxlen:
                self.curpulses = self.curpulses[1:]
                pass
            pass
        pass
    pass


