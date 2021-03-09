from rplidar import RPLidar, RPLidarException

# See: https://www.instructables.com/Getting-Started-With-the-Low-cost-RPLIDAR-Using-Je/
# https://github.com/SkoltechRobotics/rplidar

class EnvMappingService:

    # enable permisssion for port USB to which RpLidar is connected (add this to .bashrc script?)
    # sudo chmod 666 /dev/ttyUSB0
    # check with: ls -l /dev | grep ttyUSB 
    def __init__(self):
        self.lidar = RPLidar('/dev/ttyUSB0')
        #print(self.lidar.get_info())
        #print(self.lidar.get_health())
        self.lidar.clear_input()


    #each scan is a tuple (quality, angle(0-360), dist (mm))
    # returns 
    def scan(self):  
        self.lidar.stop()
        
        # clear input buffer
        self.lidar.clear_input()

        # rplidar.RPLidarException: Incorrect descriptor starting bytes
        rplidarException = False

        try:
            angle2dist = self.collect_scan_data()
        except RPLidarException:
            rplidarException = True
       
       # if exception occured try one more time...
        if rplidarException:
            self.lidar.disconnect()
            self.lidar.connect()   
            self.lidar.clear_input() 
            angle2dist = self.collect_scan_data()

        self.stop()
        return angle2dist


    def collect_scan_data(self):
        angle2dist = {} #[9999 for _ in range(360)]
        min_data_collected = False

        for i, scan in enumerate(self.lidar.iter_scans()):
            #print('%d: Got %d measurments' % (i, len(scan)))
            for tup in scan:
                ## tup ={quality,degree,distance mm}
                degree = int(round(tup[1]))%360
                distance = int(round(tup[2]))
                angle2dist[degree] = distance
                if len(angle2dist) > 345:
                    min_data_collected = True
                    break
            
            #check if collected all degrees...
            if min_data_collected:
                break
        return angle2dist


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