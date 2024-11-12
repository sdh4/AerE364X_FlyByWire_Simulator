import board

class Serial:
    device = None
    def __init__(self,port=None,baudrate=9600):
        if board.lvez1_uart == 'pyserial' and port == '/dev/ttyS0':
            import busio
            self.device = busio.LVEZ1_UART(64)
           
            pass
        pass

    def read(self,size=1):
        return self.device.read(size)

    def write(self,byts):
        return self.device.write(byts)

    @property
    def in_waiting(self):
        return self.device.in_waiting

    def read_until(self,expected=b'\n',size=None):
        output = b""
        if size == 0:
            return output
        while True:
            new = self.device.read(1)
            output += new
            if new == expected or (size is not None and len(output) >= size):
                return output
            pass
        pass
    
