from rplidar import RPLidar, RPLidarException
from threading import Thread, Event
import queue
import time

# See: https://www.instructables.com/Getting-Started-With-the-Low-cost-RPLIDAR-Using-Je/
# https://github.com/SkoltechRobotics/rplidar

class EnvMappingService:

    # enable permisssion for port USB to which RpLidar is connected (add this to .bashrc script?)
    # sudo chmod 666 /dev/ttyUSB0
    # check with: ls -l /dev | grep ttyUSB 
    def __init__(self):

        # Initialize a global lidar thread object collecting data
        self.lidarThread = None
        self.stopLidarThread = Event()
        self.angle2dist = {} 

        self.lidar = RPLidar('/dev/ttyUSB0')
        #print(self.lidar.get_info())
        #print(self.lidar.get_health())
        self.lidar.clear_input()

    def start_collision_detection(self, collision_event, path_free_event):
        if self.lidarThread is None:
            self.collision_event = collision_event
            self.path_free_event  = path_free_event
            self.lidar_scan_ready_callback = self.collision_evaluation_callback
            self.lidarThread = Thread(target=self.scan)
            self.lidarThread.start()
    
    def collision_evaluation_callback(self, data_360_sample):
        front_left = data_360_sample.get(330) 
        front_cl = data_360_sample.get(350) 
        front_center = data_360_sample.get(0) 
        front_rl = data_360_sample.get(10) 
        front_right = data_360_sample.get(30) 
        if((front_left is not None and front_left < 300) or 
           (front_cl is not None and front_cl < 300) or 
           (front_center is not None and front_center < 300) or 
           (front_rl is not None and front_rl < 300) or 
           (front_right is not None and front_right < 300)):
                # set collision event and let know other threads
                self.collision_event.set()
                self.path_free_event.clear()
        else:
            # clear collision event 
            self.path_free_event.set()
            self.collision_event.clear()

    def start_scanning(self, lidar_scan_ready_callback):
        if self.lidarThread is None:
            self.lidar_scan_ready_callback = lidar_scan_ready_callback
            self.lidarThread = Thread(target=self.scan)
            self.lidarThread.start()

    
    # notify lidar thread to stop and the waits till it finish executing
    def stop_scanning(self):
        if self.lidarThread is not None:
            self.stopLidarThread.set()
            self.lidarThread.join()
            self.lidarThread = None
            self.stopLidarThread.clear()
        self.lidar.stop()
        self.lidar.stop_motor()


    #each scan is a tuple (quality, angle(0-360), dist (mm))
    # returns 
    def scan(self):  
        while not self.stopLidarThread.is_set():
            # rplidar.RPLidarException: Incorrect descriptor starting bytes
            time.sleep(0.05)
            try:
                self.collect_scan_data()
            except RPLidarException:
                # disconnects lidar and try again
                self.lidar.disconnect()
                self.lidar.connect() 
                #self.lidar.clear_input()
                continue
                  
        self.lidar.stop()
            


    def collect_scan_data(self):
        data_collected = False
        self.angle2dist.clear()
        for i, scan in enumerate(self.lidar.iter_scans()):
            #print('%d: Got %d measurments' % (i, len(scan)))
            for tup in scan:
                ## tup ={quality,degree,distance mm}
                degree = int(round(tup[1]))%360
                distance = int(round(tup[2]))
                self.angle2dist[degree] = distance
                if len(self.angle2dist) >= 300:
                    data_collected = True
                    break
            
            #check if collected all degrees...
            if data_collected:
                self.lidar_scan_ready_callback(self.angle2dist) 
                break

            if self.stopLidarThread.is_set():
                break


    def start(self):
        print('starting lidar....')
        self.lidar.connect()
        self.lidar.clear_input()
        self.rplidar.start_motor()

    def stop(self):
        print('stopping lidar....')
        self.lidar.clear_input()
        self.lidar.stop()
        self.lidar.stop_motor()
        #self.lidar.disconnect()