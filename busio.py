import threading
import time
import board


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

class UART(object):
    device = None # The actual device we are talking to
    def __init__(self,tx=None,rx=None,*args,baudrate=9600,receiver_buffer_size=64):
        if rx.function == "PI RX" and board.lvez1_uart:
            self.device = LVEZ1_UART(receiver_buffer_size)
            pass
        pass
    
    def write(self,command):
        return self.device.write(command)
    
    def read(self,nbytes):
        return self.device.read(nbytes)

    @property
    def in_waiting(self):
        return self.device.in_waiting
    
    pass
        
class LVEZ1_UART(object):
    buffer = None # buffer of bytes available to read
    cond = None # threading.cond instance for buffer notification
    def __init__(self,receiver_buffer_size):
        self.buffer = b""
        self.receiver_buffer_size = receiver_buffer_size
        self.cond = threading.Condition()
        self.uart_thread = threading.Thread(target=self._thread_code)
        self.uart_thread.start()
        pass

    def _thread_code(self):
        import dynamic_model
        while True:
            
            time.sleep(0.05)
        
            with dynamic_model.dynamic_instance.model_lock:
                altitude_in = int(dynamic_model.dynamic_instance.pos[2]/0.0254)
                pass
            with self.cond:
                self.buffer += f"R{altitude_in:03d}\r".encode("utf-8")
                if len(self.buffer) > self.receiver_buffer_size:
                    self.buffer = self.buffer[-self.receiver_buffer_size:]
                    pass
                self.cond.notify()
                pass
    
    def write(self,command):
        return self.device.write(command)
    
    def read(self,nbytes):
        with self.cond:
            self.cond.wait_for(lambda : len(self.buffer) >= nbytes)
            got_bytes = self.buffer[:nbytes]
            self.buffer = self.buffer[nbytes:]
            pass
        return got_bytes
    @property
    def in_waiting(self):
        with self.cond:
            nbytes = len(self.buffer)
            pass
        return nbytes
    pass
