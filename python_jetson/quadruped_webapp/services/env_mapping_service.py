from rplidar import RPLidar, RPLidarException
from threading import Thread, Event
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
            self.lidar.stop_motor()


    #each scan is a tuple (quality, angle(0-360), dist (mm))
    # returns 
    def scan(self):  
        while not self.stopLidarThread.is_set():
            # rplidar.RPLidarException: Incorrect descriptor starting bytes
            rplidarException = False

            try:
                self.collect_scan_data()
            except RPLidarException:
                rplidarException = True
        
            # if exception occured disconnects and try one more time...
            if rplidarException:
                self.lidar.disconnect()
                self.lidar.connect()   
                self.collect_scan_data()

            self.lidar.stop()
            time.sleep(0.2)


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
                if len(self.angle2dist) >= 330:
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