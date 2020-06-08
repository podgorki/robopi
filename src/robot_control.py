 #!/usr/bin/env python   
import time
import serial


class robotControl():
    
    COMMAND = {'start': chr(128), 'baud': chr(129), 'control': chr(130), 'safe': chr(131), 'full': chr(132), 'power': chr(133),
               'spot': chr(134), 'clean': chr(135), 'max': chr(136), 'drive': chr(137), 'motors': chr(138), 'led': chr(139),
               'song': chr(140), 'play': chr(141), 'sensors': chr(142), 'force-seek-dock': chr(143)
               }
    
    SENSORS = {}
    
    MOTORS = {'none': chr(0), 'side': chr(1), 'vacuum': chr(2), 'main': chr(4), 'all': chr(7)}
    
    def __init__(self):
        self.ser = serial.Serial(
                                port='/dev/ttyS0',
                                baudrate = 57600,
                                bytesize = 8,
                                stopbits=1
                                )
        
    def send(self, byte):
        self.ser.write(byte.encode())
        return
    
    def toTwosComplement2Bytes(self, value ):
        """ returns two bytes (ints) in high, low order
        whose bits form the input value when interpreted in
        two's complement
        """
        # if positive or zero, it's OK
        if value >= 0:
            eqBitVal = value
        # if it's negative, I think it is this
        else:
            eqBitVal = (1<<16) + value
        
        return ( chr((eqBitVal >> 8) & 0xFF), chr(eqBitVal & 0xFF ))

    def activate_robot_control(self):
        self.send(byte=self.COMMAND['start']) # put roomba in passive mode
        time.sleep(0.25) # wait
        self.send(byte=self.COMMAND['control']) # put roomba in safe mode - allows robot control
        time.sleep(0.25) # wait
        self.send(byte=self.COMMAND['safe']) # put roomba in safe mode - allows robot control
        time.sleep(0.25) # wait
        self.send(byte=self.COMMAND['full']) # put roomba in full mode - allows unrestricted robot control
        time.sleep(0.5)
        return
    
    def drive(self, ave_velocity_mm_p_sec: int = 0, radius: int = 32768):
        """
        This function commands the roomba to drive.
        ave_velocity_mm_p_sec: int values between -500 and 500 mm/s (default=0 i.e. stopped)
        radius: int value between -2000 - 2000 mm (default=32768 - special case of straight)
        Drive Command opcode: 137 Number of data bytes: 4
        Controls Roomba’s drive wheels. The command takes four data
        bytes, which are interpreted as two 16 bit signed values using
        twos-complement. The first two bytes specify the average velocity
        of the drive wheels in millimeters per second (mm/s), with the
        high byte sent first. The next two bytes specify the radius, in
        millimeters, at which Roomba should turn. The longer radii make
        Roomba drive straighter; shorter radii make it turn more. A Drive
        command with a positive velocity and a positive radius will make
        Roomba drive forward while turning toward the left. A negative
        radius will make it turn toward the right. Special cases for the
        radius make Roomba turn in place or drive straight, as specified
        below. The SCI must be in safe or full mode to accept this
        command. This command does change the mode
        """
        velHighVal, velLowVal = self.toTwosComplement2Bytes(value=ave_velocity_mm_p_sec)
        radHighVal, radLowVal = self.toTwosComplement2Bytes(value=radius)
        self.send(byte=self.COMMAND['drive'])
        self.send(byte=velHighVal)
        self.send(byte=velLowVal)
        self.send(byte=radHighVal)
        self.send(byte=radLowVal)
        return
    
    def cleaning_motors(self, motor: str = 'none'):
        """
        motor: str option from MOTORS default is motors off i.e. none
        Controls Roomba’s cleaning motors. The state of each motor is
        specified by one bit in the data byte. The SCI must be in safe
        or full mode to accept this command. This command does not
        change the mode.
        """
        self.send(byte=self.COMMAND['motors'])
        self.send(byte=self.MOTORS[motor])
        return

robopi = robotControl()
robopi.activate_robot_control()

for i in range(5):
    robopi.cleaning_motors(motor='side') # currently doesnt do anything...
    robopi.drive(ave_velocity_mm_p_sec=10)
    time.sleep(5)
    robopi.cleaning_motors()
    robopi.drive()  # default is stopped
    time.sleep(5)
