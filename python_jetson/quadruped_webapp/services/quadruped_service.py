import serial
import glob
import time
import RPi.GPIO as GPIO
from threading import Thread, Event


class QuadrupedService:

    CMD_TERMINATOR = "|"
    CMD_EMPTY = ""
    CMD_STATUS = "STATUS"
    CMD_WALK_FORWARD = "WALK_FORWARD"
    CMD_WALK_BACKWARD = "WALK_BACKWARD"
    CMD_STOP = "STOP"
    CMD_TURN_LEFT = "TURN_LEFT"
    CMD_TURN_RIGHT = "TURN_RIGHT"
    CMD_GREET = "GREET"
    CMD_FOOTWORK = "FOOTWORK"
    CMD_SWING = "SWING"

    def __init__(self):
        try:
            self.init_serial_to_arduino('/dev/ttyACM0')
        except Exception as e: 
            self.init_serial_to_arduino('/dev/ttyACM1')
       
        #setup GPIO LED light
        self.led_control_pin = 16
        self.led_vcc_pin = 18
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.led_control_pin, GPIO.OUT) #led control (ON/OFF)
        GPIO.setup(self.led_vcc_pin, GPIO.OUT) #led Vcc (5V)

    def init_serial_to_arduino(self, port_name):
        self.arduino = serial.Serial(
                port = port_name,
                baudrate = 115200,
                bytesize = serial.EIGHTBITS, 
                parity = serial.PARITY_NONE,
                stopbits = serial.STOPBITS_ONE, 
                timeout = 10,
                xonxoff = False,
                rtscts = False,
                dsrdtr = False,
                write_timeout=None
                #writeTimeout = 5
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
            #time.sleep(0.2) # give some time for Arduino...
            #resonse = self.arduino.read
            #response = self.arduino.read_until(self.CMD_TERMINATOR).decode()
            #if response:
            #    print("arduino response: " + response)
        except Exception as e:
            print(e)
            #arduino.close() 
    
    def get_arduino_status(self):
        arduino_status = ""
        try:
            print("sending command: " + command + " to Arduino MCU board...")
            self.arduino.write((command + self.CMD_TERMINATOR).encode()) 
            time.sleep(5) # give some time for Arduino...
            arduino_status = self.arduino.read_until(self.CMD_TERMINATOR).decode()
        except Exception as e:
            print(e)
        return arduino_status


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
    
    def blink_light(self):
        try:
            GPIO.output(self.led_vcc_pin, GPIO.HIGH)
            curr_state = GPIO.HIGH
            for i in range(10):
                GPIO.output(self.led_control_pin, curr_state)
                time.sleep(0.3)
                curr_state ^= GPIO.HIGH
        finally:
            GPIO.output(self.led_vcc_pin, GPIO.LOW)
            GPIO.output(self.led_control_pin, GPIO.LOW)
            #GPIO.cleanup()

    

    def robot_worker_thread(self, collision_event, stop_event):
        is_walking = False
        is_turning = False
        GPIO.output(self.led_vcc_pin, GPIO.HIGH)
        GPIO.output(self.led_control_pin, GPIO.LOW)
        # check for collision every 200 ms
        collision_check_timeout = 0.2 # 200 ms
        while not stop_event.is_set():
            if(collision_event.wait(collision_check_timeout)):
                GPIO.output(self.led_control_pin, GPIO.HIGH) # indicate collision with front LED light
                if(is_turning==False):
                    self.turn_right() # when collission detected keep turning
                    is_turning = True
                    is_walking = False
            else:
                GPIO.output(self.led_control_pin, GPIO.LOW)
                if(is_walking==False):
                    self.walk() # walk forward
                    is_walking = True
                    is_turning = False



    def free_walk_with_collision_avoidance(self, env_mapping_service):
        #stop scanning first
        env_mapping_service.stop_scanning()
        time.sleep(1)

        # start robot worker thread
        self.collision_event = Event()
        self.stop_free_walk_event = Event()
        self.robot_worker_thread = Thread(target=self.robot_worker_thread, args=(self.collision_event, self.stop_free_walk_event))
        self.robot_worker_thread.start()

        # start lidar thread (through env mapping service)
        env_mapping_service.start_collision_detection(self.collision_event)


    def stop_free_walk(self, env_mapping_service):
        if self.robot_worker_thread is not None:
            if self.stop_free_walk_event is not None:
                self.stop_free_walk_event.set()
                self.stop_free_walk_event.clear()
            self.robot_worker_thread.join()
            self.robot_worker_thread = None
            # stop scanning
            env_mapping_service.stop_scanning()
            # put robot into home posistion (stop)
            self.homePosition()









