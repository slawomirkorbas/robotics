import serial
import glob
import time

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
            timeout = 10,
            xonxoff = False,
            rtscts = False,
            dsrdtr = False,
            writeTimeout = 10
        )

    def shut_down(self):
        print('Closing port... ')
        try:
            self.arduino.close()
        except Exception as e:
            print(e)


    def send_cmd_2_arduino(self,command):
        try:
            print("sending command: " + command + " to Arduino MCU board...")
            self.arduino.write((command + self.CMD_TERMINATOR).encode()) 
            time.sleep(0.2) # givee some time for Arduino...
            response = self.arduino.read_until(self.CMD_TERMINATOR).decode()
            if response:
                print("arduino response: " + response)
        except Exception as e:
            print(e)
            #arduino.close() 

    def homePosition(self):
        return self.send_cmd_2_arduino(self.CMD_STOP)

    def walk(self):
        return self.send_cmd_2_arduino(self.CMD_WALK_FORWARD)

    def swing(self):
        return self.send_cmd_2_arduino(self.CMD_SWING)

    def greet(self):
        return self.send_cmd_2_arduino(self.CMD_GREET)

    def turn_right(self):
        return self.send_cmd_2_arduino(self.CMD_TURN_RIGHT)

    def turn_left(self):
        return self.send_cmd_2_arduino(self.CMD_TURN_LEFT)