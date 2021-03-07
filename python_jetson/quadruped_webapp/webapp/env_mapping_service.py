from rplidar import RPLidar

# See: https://www.instructables.com/Getting-Started-With-the-Low-cost-RPLIDAR-Using-Je/
# https://github.com/SkoltechRobotics/rplidar

class EnvMappingService:

    # enable permisssion for port USB to which RpLidar is connected (add this to .bashrc script?)
    # sudo chmod 666 /dev/ttyUSB0
    # check with: ls -l /dev | grep ttyUSB 
    def __init__(self):
        self.lidar = RPLidar('/dev/ttyUSB0')
        print(self.lidar.get_info())
        print(self.lidar.get_health())
        self.lidar.clear_input()


    #each scan is a tuple (quality, angle(0-360), dist (mm))
    def scan(self, no_of_samples):  
        self.lidar.stop()
        result = ''
        for i, scan in enumerate(self.lidar.iter_scans()):
            print('%d: Got %d measurments' % (i, len(scan)))
            result +=  str(i) + " " + str(scan) + "\r\n"
            if i > no_of_samples:
                break
        return result


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