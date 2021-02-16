import serial

class QuadrupedService:

    CMD_TERMINATOR = "|"
    CMD_EMPTY = ""
    CMD_WALK_FORWARD = "WALK_FORWARD"
    CMD_WALK_BACKWARD = "WALK_BACKWARD"
    CMD_STOP = "STOP"
    CMD_TURN_LEFT = "TURN_LEFT"
    CMD_TURN_RIGHT = "TURN_RIGHT"
    CMD_GREET = "GREET"
    CMD_FOOTWORK = "FOOTWORK"
    CMD_SWING = "SWING"

    def __init__(self):
        self.arduino = serial.Serial(
            port = '/dev/ttyACM0',
            baudrate = 115200,
            bytesize = serial.EIGHTBITS, 
            parity = serial.PARITY_NONE,
            stopbits = serial.STOPBITS_ONE, 
            timeout = 5,
            xonxoff = False,
            rtscts = False,
            dsrdtr = False,
            writeTimeout = 2
        )

    def send_cmd_2_arduino(self,command):
        try:
            print("sending command: " + command + " to Arduino MCU board...")
            self.arduino.write((command + self.CMD_TERMINATOR).encode()) 
            response = self.arduino.readline()
            if response:
                print("arduino response: " + response)
        except Exception as e:
            print(e)
            #arduino.close() 

    def homePosition(self):
        return self.send_cmd_2_arduino(self.CMD_STOP)