lvez1_uart = None # Alternatives are "UART" for circuit python or "pyserial" for RaspberryPi
bmp3xx_i2c = False

class pin(object):
    function = None

    def __init__(self,function):
        self.function=function
        pass
    pass

# This file defines the pins on the
# simulated Featherboard or Raspberry PI

# Basically, in the dynamic model you can
# access the objects that use these pins
# based on an assumption of what module
# is used to access them

# For example if main.py is using the
# pin named LED with the
# digitalio.DigitalInOut class, then that
# the dynamic_model can access that object
# via digitalio.DigitalInOuts_by_pin["LED"]
#
# Please note that outputs from main.py
# are the inputs of the dynamic model,
# and outputs of the dynamic model are
# inputs of main.py


D13=pin("LED")
D5=pin("ROT CMD")
#D6=pin("FWD CMD")  # Featherboard FWD CMD
D6 = pin("LIFT CMD")
D12=pin("FWD CMD")   # RPI FWD CMD
SCL=pin("SCL")
SDA=pin("SDA")

D9=pin("LEFT OUT")
D10=pin("RIGHT OUT")
D11 = pin("LIFT OUT")

# These are for the Featherboard?
#D7=pin("LEFT OUT") 
#D8=pin("RIGHT OUT")

D13=pin("LED")

TX=pin("TX")
RX=pin("RX")

