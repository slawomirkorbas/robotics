// Sketch for custom  Quadruped robot based on Arduino Leonardo + Dynamixel Shield(Robotis) controlled by NVIDIA Jetson Nano onboard computer. 

// HOW IT WORKS
// The porogram waits for commands send over Serial port(USB) from Jetson Nano (The special Python application has to be launched on Jetson Nano).
// Each command corresponds to specific robot activity like : walking, balancing or changing movement directions. These activities are declared as string literals on both sides: Jetson(master) and Arduino (Slave).
// Once the command is received then the robot directs servomotors to perform specific sequence. While executing commands the program pools the Serial input as often as possible(*) in order to swich to another activity(mode).
// 
// HARDWARE
// Arduino Leonardo: https://www.arduino.cc/en/Main/Arduino_BoardLeonardo
// Dynamixel Shield: https://emanual.robotis.com/docs/en/parts/interface/dynamixel_shield/
// NVIDIA Jetson Nano: https://developer.nvidia.com/embedded/jetson-nano-developer-kit
// Servomotors XL430-W-250T (12 modules):  https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/
// Gyroscope MPU6050: https://www.dfrobot.com/product-880.html
// Other connected to Jetson Nano (not Arduino): RPLIDAR: http://www.slamtec.com/en/lidar/a1
//
// LIBRARIES:
// DynamixelShield: https://www.arduino.cc/reference/en/libraries/dynamixelshield/
// Dynamixel2Arduino: https://www.arduino.cc/reference/en/libraries/dynamixel2arduino/
// I2CDev + MPU6050:  https://image.dfrobot.com/image/data/SEN0142/6%20Dof%20-%20MPU6050%20library.zip  
//
// 2021-05-22 by Slawomir Korbas <slawomirkorbas@gmail.com>


#include <DynamixelShield.h>
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DEBUG_SERIAL soft_serial
#elif defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAM_ZERO)
  #define DEBUG_SERIAL SerialUSB    
#else
  #define DEBUG_SERIAL Serial
#endif

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// Gyroscope module #####################################################################################################################################################
MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

// Dynamixel servo control module ########################################################################################################################################
const float DXL_PROTOCOL_VERSION = 2.0;
DynamixelShield shield;
//This namespace is required to use Control table item names
using namespace ControlTableItem;
// #######################################################################################################################################################################

/**
 * Wrapper class for dynamixel servo motor
 */
class DynamixelServo
{
  public:
    int id;

    DynamixelServo(int servoId) {
      this->id = servoId;
    }

    String getStatus() {
        String servoStatus = "---------------------------------\r\n";
        uint32_t  timeout = 300; //ms
        servoStatus += "Dynamixel servo " + String(this->id) + " status info: \r\n";
        servoStatus += "MODEL_INFORMATION: " + String(shield.readControlTableItem(MODEL_INFORMATION, id, timeout)) + "\r\n";
        servoStatus += "PRESENT_VOLTAGE: " + String(shield.readControlTableItem(PRESENT_VOLTAGE, id, timeout)) + "r\n";
        servoStatus += "PRESENT_POSITION: " + String(shield.readControlTableItem(PRESENT_POSITION, id, timeout)) + "\r\n";
        int32_t hardware_err_status = shield.readControlTableItem(HARDWARE_ERROR_STATUS, id, timeout);
        servoStatus += "HARDWARE_ERROR_STATUS: " + String(hardware_err_status) + "\r\n";
        if(hardware_err_status & (1 << 5) ) {
          servoStatus += "ERROR: Bit 5  Overload Error(default) Detects that persistent load that exceeds maximum output \r\n";
        }
        if(hardware_err_status & (1 << 4)) {
            servoStatus += "ERROR:  Bit 4 Electrical Shock Error(default) Detects electric shock on the circuit or insufficient power to operate the motor \r\n";
        }
        if(hardware_err_status & (1 << 3)) {
            servoStatus += "ERROR:  Bit 3 Motor Encoder Error Detects malfunction of the motor encoder \r\n";
        }
        if(hardware_err_status & (1 << 2)) {
            servoStatus += "ERROR:  Bit 2 Overheating Error(default)  Detects that internal temperature exceeds the configured operating temperature \r\n";
        }
        if(hardware_err_status & (1 << 0)) {
            servoStatus += "ERROR:  Bit 0 Input Voltage Error Detects that input voltage exceeds the configured operating voltage \r\n";
        }
        return servoStatus;
    }

    void setHomingOffset(double offsetDeg) {
      uint32_t offset = offsetDeg/0.088; // Unit 0.088
      uint32_t writeTimeout = 100; // ms
      shield.writeControlTableItem(HOMING_OFFSET, id, offset, writeTimeout);
    }


    void setOperatingMode(int mode) {
          shield.torqueOff(id);
          shield.setOperatingMode(id, mode);
          shield.torqueOn(id);
    }

     /** 
      *  Velocity can be in range: 0 - 1023. Before sending it to Dynamixel's control table PROFILE_VELOCITY it has to be multiplicated by 0.229 rpm - what is the unit for this value.
      */
     boolean setSpeed(float speedPct) {
          double maxDynamixelSpeed = 1023 * 0.229; //RPM
          uint32_t newSpeedRpm = speedPct * maxDynamixelSpeed / 100;
          uint32_t writeTimeout = 100; // ms
          shield.writeControlTableItem(PROFILE_VELOCITY, id, newSpeedRpm, writeTimeout);
     }

     boolean ping() {
      if(!shield.ping(id)) {
           //Serial.print("Failed to ping servo");
           //Serial.println(id);
       }
       else {
           //Serial.print("Servo ping successfull: ");
           //Serial.println(id);
       }
     }             

     void homePosition() {
        shield.setGoalPosition(id, 180, UNIT_DEGREE);
     }

     void setAngleRelative(float offsetDeg) {
       shield.setGoalPosition(id, 180 + offsetDeg, UNIT_DEGREE);
     }

     void setAngle(float angle) {
        shield.setGoalPosition(id, angle, UNIT_DEGREE);
     }
};


/**
 * Class representing specific motion frame for robot joint.
 * 3DOF Joint angles (hip,femur,tibia) relative to home servo's position are calculated using geometric inverse kinematic formulas.
 * As reference see: https://appliedgo.net/roboticarm/
 */
class KeyFrame {
  public:
    float x, y, z;
    KeyFrame(float x, float y, float z){
      this->x = x;
      this->y = y;
      this->z = z;
    }

    void transform() {
      // to implement
    }
};

/**
 * Class representing motion sequence consisting of multiple key frames 
 */
class MotionSequnce {
    private:
      KeyFrame* motionFrames;
      int len;
  
    public:
      MotionSequnce( KeyFrame* frames, unsigned int len ) {
        this->motionFrames = frames;
        this->len = len; 
      }

      KeyFrame next(int& current) {
        current = current >= len ? 0 : current;
        return motionFrames[ current++ ];
      }   
};

/**
 * Class representing robot leg consisnting of 3 servos (joints)
 */
class Leg
{
   private:
      boolean inv;
      DynamixelServo *servos[3];
      DynamixelServo *hip;
      DynamixelServo *femur;
      DynamixelServo *tibia;

   public:
     Leg(int s1, int s2, int s3, boolean inverted) {
        servos[0] = new DynamixelServo(s1);
        hip = servos[0];
        servos[1] = new DynamixelServo(s2);
        femur = servos[1];
        servos[2] = new DynamixelServo(s3);
        tibia = servos[2];
        this->inv = inverted;
     }

     DynamixelServo* getHip(){return this->hip;};
     DynamixelServo* getFemur(){return this->femur;};
     DynamixelServo* getTibia(){return this->tibia;};
  
     void enablePositionMode() {
        for(int i=0; i<3; i++) {
          servos[i]->setOperatingMode(OP_POSITION);
        }
     }

     boolean setSpeed(float speedPct) {
          for(int i=0; i<3; i++) {
             servos[i]->setSpeed(speedPct); 
          }
     }

    void moveTo(KeyFrame keyFrame) {
        moveTo(keyFrame.x, keyFrame.y, keyFrame.z);
    }

    void moveTo(float targetX, float targetY, float targetZ) {
        float hipAngle, femurAngle, tibiaAngle;
        float femurLen = 9.6;  //cm
        float tibiaLen = 11.0; //cm
        float d1 = sqrt(sq(targetZ) + sq(targetY)); 
        hipAngle = toDeg(atan(targetZ/targetY));

        float d2 = sqrt(sq(d1) + sq(targetX));
        tibiaAngle = angleFromCosineRuleDeg(femurLen, tibiaLen, d2);

        float beta1 = angleFromCosineRuleDeg(femurLen, d2, tibiaLen);
        float beta2 = toDeg(atan(targetX/d1));
        femurAngle = abs(beta1 - beta2);
        
        hip->setAngle(180 - hipAngle);
        femur->setAngle(this->inv ? 180 - femurAngle : 180 + femurAngle);
        tibia->setAngle(this->inv ? 360 - tibiaAngle : tibiaAngle);
    }

     /*
     * Calculates angle using law of cosine for known triangle edges
     */
    float angleFromCosineRuleDeg(float a, float b, float c) {
      float radAngle = safe_acos((sq(a) + sq(b) - sq(c))/(2*a*b));
      return toDeg(radAngle);
    }

    float toDeg(float rad){
      return (rad * (180/3.1415));
    }
    
    
    double safe_acos(double value) {
        if (value<=-1.0) {
            return 3.1415;
        } else if (value>=1.0) {
            return 0;
        } else {
            return acos(value);
        }
    }

     void ping() {
         for(int i=0; i<3; i++) {
            servos[i]->ping();
         }
     }

     String getStatus() {
        String legStatus = "";
        for(int i=0; i<3; i++) {
            legStatus += servos[i]->getStatus();
         }
        return legStatus;
     }
     
};


/** Command set for the robot **/
static const char TERMINATOR = '|';
static const String CMD_EMPTY = "";
static const String CMD_STATUS = "STATUS";
static const String CMD_WALK_FORWARD = "WALK_FORWARD";
static const String CMD_WALK_BACKWARD = "WALK_BACKWARD";
static const String CMD_STOP = "STOP";
static const String CMD_TURN_LEFT = "TURN_LEFT";
static const String CMD_TURN_RIGHT = "TURN_RIGHT";
static const String CMD_GREET = "GREET";
static const String CMD_FOOTWORK = "FOOTWORK";
static const String CMD_SWING = "SWING";


/**
 * Main Quadruped robot class.
 */
class QuadrupedRobot {
  public:
    Leg leg_fR = Leg(10, 11, 12, true);  
    Leg leg_rR = Leg(31, 30, 32, true);
    Leg leg_fL = Leg(20, 21, 22, false);
    Leg leg_rL = Leg(41, 40, 55, false);  
    Leg legs[4] = {leg_fR, leg_rR, leg_fL, leg_rL };

    String currenlyExecutedCommand;

    QuadrupedRobot()
    {
      
    }


    /** Check if there is new order from Jetson and reads it from the buffer **/
    String getNextCommandFromJetson() {
        String commandFromJetson = CMD_EMPTY;
        int bytes = Serial.available();
        if(bytes >= 5) 
        {
            commandFromJetson = Serial.readStringUntil(TERMINATOR);
            cleanUpInputBuffer(); // removes remaining data from input buffer
            String ackMsg = "New command accepted: " + commandFromJetson; 
            currenlyExecutedCommand = commandFromJetson;
        }
        return commandFromJetson;
    }


    /** clean all data from input buffer **/
    String cleanUpInputBuffer() {
        while(Serial.available() > 0) {
          char t = Serial.read();
        }
    }

    /** returns robot status **/
    String robotStatus() {
        String status = "Robot status: \r\n\\r\n";
        for( int i=0; i<4; i++) {
            status += legs[i].getStatus();
        }
        return status;
    }

    /**
    * Servo smoke test and configuration...
    */
    void configure() 
    {
        for( int i=0; i<4; i++) {
          legs[i].ping();
          legs[i].enablePositionMode();
        }
    }

    void setSpeed(double percentage)
    {
      for( int i=0; i<4; i++) {
          legs[i].setSpeed(percentage);
      }
    }


    String stop() {
        this->homePosition();
        String nextCommand = CMD_EMPTY;
        while(nextCommand == CMD_EMPTY) {
            delay(500);
            nextCommand = getNextCommandFromJetson();
        }
        return nextCommand;
    }

    void homePosition()
    {
         leg_rL.moveTo(1, 15, 0);
         leg_fL.moveTo(1, 15, 0);
         leg_rR.moveTo(1, 15, 0);
         leg_fR.moveTo(1, 15, 0);   
    }


    String greet(){
      this->homePosition();
      delay(1000);
      
          leg_rL.moveTo({5,11,0});
          leg_rR.moveTo({5,11,0});
          delay(1000);
          leg_fL.moveTo({1,19,-2});
          delay(1000);
          leg_fR.getFemur()->setAngle(250);
          leg_fR.getTibia()->setAngle(200);
          delay(3000);
          leg_fR.moveTo(1, 16, 0); 
          leg_fL.moveTo(1, 16, 0); 
          delay(2000);

        return CMD_STOP;
    }
  

    String walk()
    {
       int wait = 200;
       KeyFrame frames[] = { {-5,15,0}, {-5,15,0}, {3,12,0}, {5,15,0} }; 
       MotionSequnce motionSeqence( frames, sizeof(frames)/sizeof(frames[0]) );
       int fL=2, fR=0, rL=0, rR=2; // starting frame indexes for each leg
       String nextCommand = CMD_EMPTY;
       while(nextCommand == CMD_EMPTY) {
          delay(wait);
          leg_fL.moveTo(motionSeqence.next(fL));
          leg_rL.moveTo(motionSeqence.next(rL));
          leg_fR.moveTo(motionSeqence.next(fR));
          leg_rR.moveTo(motionSeqence.next(rR));   
          nextCommand = getNextCommandFromJetson();
       }
       return nextCommand; 
    }


    String walk_2() {
       int wait = 500;
       String nextCommand = CMD_EMPTY;
       while(nextCommand == CMD_EMPTY) {
         delay(wait);
         leg_fL.moveTo(3,12,0);
         leg_rR.moveTo(3,12,0);
         leg_fR.moveTo(1,15,0);
         leg_rL.moveTo(1,15,0); 
         delay(wait);
         leg_fL.moveTo(5,15,0);
         leg_rR.moveTo(5,15,0);
         leg_fR.moveTo(-4,16,0);
         leg_rL.moveTo(-4,16,0); 
         delay(wait);
         leg_fL.moveTo(1,15,0);
         leg_rR.moveTo(1,15,0);
         leg_fR.moveTo(3,12,0);
         leg_rL.moveTo(3,12,0); 
         delay(wait);
         leg_fL.moveTo(-4,16,0);
         leg_rR.moveTo(-4,16,0);
         leg_fR.moveTo(5,15,0);
         leg_rL.moveTo(5,15,0); 
         nextCommand = getNextCommandFromJetson();
       }
       return nextCommand;
    }

    String turn(boolean right) {    
         KeyFrame fr_f_01[] = { {1,12,-2}, {1,14,-2}, {1,14,-2}, {1,14,0} }; 
         KeyFrame fr_f_02[] = { {1,14,0 }, {1,14,0 }, {1,12,2 }, {1,14,2} };     
         int f_seq_len = sizeof(fr_f_01)/sizeof(fr_f_01[0]);
         MotionSequnce m_seq_f1( fr_f_01, f_seq_len );
         MotionSequnce m_seq_f2( fr_f_02, f_seq_len );
         int wait = 150;
         int rr_01=0, rr_02=0, ff_01=0, ff_02=0; // starting frame indexes for each leg
         String nextCommand = CMD_EMPTY;
         while(nextCommand == CMD_EMPTY) {
              delay(wait); 
              leg_fL.moveTo(right ? m_seq_f1.next(ff_01) : m_seq_f2.next(ff_01));
              leg_fR.moveTo(right ? m_seq_f2.next(ff_02) : m_seq_f1.next(ff_02)); 
              leg_rL.moveTo(right ? m_seq_f2.next(rr_01) : m_seq_f1.next(rr_01));
              leg_rR.moveTo(right ? m_seq_f1.next(rr_02) : m_seq_f2.next(rr_02)); 
              nextCommand = getNextCommandFromJetson();
         }
        return nextCommand;
  }
    
};

/** Create robot object **/
QuadrupedRobot robot = QuadrupedRobot();

void setup() {
 
  setupSerialCommunication();
  setupMPU6050Gyroscope();
  setupDynamixelShield();

   delay(2000);
  robot.configure();
  robot.setSpeed(20.0);
}

void setupSerialCommunication() {
    // initialize serial communication
    Serial.begin(115200, SERIAL_8N1); // For Uno, Nano, Mini, and Mega, use UART port of DYNAMIXEL Shield to debug.
    while (!Serial); // wait for Leonardo enumeration, others continue immediately
}

void setupDynamixelShield() {
   shield.begin(57600);                                    // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
   shield.setPortProtocolVersion(DXL_PROTOCOL_VERSION);    // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
}

void setupMPU6050Gyroscope() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
}

// ################################################################################ Main program loop ##########################################################################################

String nextCmd = CMD_STOP;
void loop() 
{ 
  //robot.leg_rL.getFemur()->setAngle(180);
  
  if(!nextCmd.equals(CMD_EMPTY)) 
  {
      if(CMD_STOP.equals(nextCmd))
      {
          nextCmd = robot.stop();
      }
      if(CMD_WALK_FORWARD.equals(nextCmd))
      {
          nextCmd = robot.walk_2();
      }
      if(CMD_TURN_LEFT.equals(nextCmd))
      {
          nextCmd = robot.turn(true);
      }
      if(CMD_TURN_RIGHT.equals(nextCmd))
      {
          nextCmd = robot.turn(false);
      }    
      if(CMD_GREET.equals(nextCmd))
      {
          nextCmd = robot.greet();
      }  
  }

}
