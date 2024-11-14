
AerE 3640X Fly-By-Wire Simulator
--------------------------------
  * Requires Python and numpy/matplotlib
  * Your model should get saved as "main.py"
    in this folder.
  * See "main_template.py" for a starting point
  * Run the simulator from the command line or Anaconda
    prompt from this folder/directory as:
      "python simulator.py"
  * The simulator will then initialize itself and
    start running "main.py" 
  * Debug by paying attention to any exceptions
    raised and by adding print() calls to your code.

Simulated craft
---------------
The simulated craft has a single lift fan and left and right
forward facing fans for directional control and forward motion.
The left and right fans are on reversing speed controllers so
they are in their null position (no thrust) with a 1.5 ms
pulse. The lift fan is on a uni-directional speed controller that
is in its null position with a 1.0 ms pulse.

The Fly-By-Wire code receives rotation command on GPIO D5 (board.D5)
It receives its lift command on GPIO D6 (board.D6)
It receives its forward command on GPIO D12 (board.D12)
It has an I2C bus with a BNO055 imu on pins board.SCL and
board.SDA. There is an optional maxbotics LV-EZ1 rangefinder for
altitude attached to the serial port (board.RX).

The Fly-By-Wire code can command thrust on the left, right,
and lift fans by using pwmio on pins board.D9, board.D10, and
board.D11, respectively. It can also turn on an LED by using GPIO
pin D13 as an output.

Configuration
-------------
You can enable "wind" that imparts a turning moment to the craft by:
  import dynamic_model
  dynamic_model.enable_wind=True

You can enable plotting and controlling the vertical axis with 
  import dynamic_model
  dynamic_model.enable_vertical_motion=True

You can enable a simulated Maxbotix LV-EZ1 rangefinder for
altitude with:
  import board
  board.lvez1_uart="UART" # For CircuitPython compatibility, or
  board.lvez1_uart="pyserial" # For Raspberry Pi compatibility
On simulated CircuitPython you communicate using busio.UART().
On simulated Raspberry Pi you:
  import serial
  rf=serial.Serial("/dev/ttyS0",baudrate=9600)

You can enable a simulated Adafruit BMP390 pressure altitude sensor
with:
  board.bmp3xx_i2c=True




  

