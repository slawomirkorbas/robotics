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

/**
 * In case of problems with port access on Ubuntu (JETSON NANO):
 * ls -l /dev/ttyACM*, this will show the group that has access to the port.
 * THEN:
 * sudo adduser YourUserName GroupToJoin  Will fix things   (the group may be called eg: dialout)
 * 
 * ALSO call: sudo chmod a+rw /dev/ttyACM0
 */


const float DXL_PROTOCOL_VERSION = 2.0;

DynamixelShield shield;

//This namespace is required to use Control table item names
using namespace ControlTableItem;

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

    void printStatus() {
        uint32_t  timeout = 300; //ms
        Serial.print("Dynamixel servo ");
        Serial.print(id);
        Serial.println(" status info: ");
        Serial.print("MODEL_INFORMATION: ");
        Serial.println(shield.readControlTableItem(MODEL_INFORMATION, id, timeout));
        Serial.print("PRESENT_VOLTAGE: ");
        Serial.println(shield.readControlTableItem(PRESENT_VOLTAGE, id, timeout));
        Serial.print("PRESENT_POSITION: ");
        Serial.println(shield.readControlTableItem(PRESENT_POSITION, id, timeout));
  
        Serial.print("HARDWARE_ERROR_STATUS: ");
        int32_t hardware_err_status = shield.readControlTableItem(HARDWARE_ERROR_STATUS, id, timeout);
        Serial.println(hardware_err_status);
        if(hardware_err_status & (1 << 5) ) {
          Serial.println("ERROR: Bit 5  Overload Error(default) Detects that persistent load that exceeds maximum output");
        }
        if(hardware_err_status & (1 << 4)) {
          Serial.println("ERROR:  Bit 4 Electrical Shock Error(default) Detects electric shock on the circuit or insufficient power to operate the motor");
        }
        if(hardware_err_status & (1 << 3)) {
          Serial.println("ERROR:  Bit 3 Motor Encoder Error Detects malfunction of the motor encoder");
        }
        if(hardware_err_status & (1 << 2)) {
          Serial.println("ERROR:  Bit 2 Overheating Error(default)  Detects that internal temperature exceeds the configured operating temperature");
        }
        if(hardware_err_status & (1 << 0)) {
          Serial.println("ERROR:  Bit 0 Input Voltage Error Detects that input voltage exceeds the configured operating voltage");
        }
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
           Serial.print("Failed to ping servo");
           Serial.println(id);
       }
       else {
           Serial.print("Servo ping successfull: ");
           Serial.println(id);
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
        DEBUG_SERIAL.println(" ");
        DEBUG_SERIAL.print("angle ");
        DEBUG_SERIAL.print(angle);
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
 
     void moveTo(float targetX, float targetY) {

        float femurAngle, tibiaAngle;
        float femurLen = 9.6;  //cm
        float tibiaLen = 11.0; //cm
        float dist = sqrt(sq(targetX) + sq(targetY));
        tibiaAngle = angleFromCosineRuleDeg(femurLen, tibiaLen, dist);
        float beta1 = angleFromCosineRuleDeg(dist, femurLen, tibiaLen);
        float beta2 = toDeg(atan(targetX/targetY));
        femurAngle = abs(beta2 - beta1);
        
          Serial.println("-----");
          Serial.print("beta1 ");
          Serial.println(beta1);
          Serial.print("beta2 ");
          Serial.println(beta2);

        //hip->setAngle(hipAngle);
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
            servos[i]->printStatus();
         }
     }
     
};


/** Command set for the robot **/
static const String CMD_EMPTY = "";
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


    String currenlyExecutedCommand;

    QuadrupedRobot()
    {
      
    }


    /** Check if there is new order from Jetson and reads it from the buffer **/
    String getNextCommandFromJetson() {
        String commandFromJetson = CMD_EMPTY;
        static const char TERMINATOR = '|';
        int numberOfBytes = Serial.available();
        if(numberOfBytes > 0)
        {
          commandFromJetson = Serial.readStringUntil(TERMINATOR);
          // confirm 
          String ackMsg = "New command: " + commandFromJetson + ". Bytes received: " + String(numberOfBytes) + ". Previous command: " + currenlyExecutedCommand + TERMINATOR; 
          Serial.print(ackMsg);
          //Serial.flush();
          currenlyExecutedCommand = commandFromJetson;
        }
        return commandFromJetson;
    }

    /**
    * Servo smoke test and configuration...
    */
    void configure() 
    {
        Serial.println("Robot configuration started...");
        leg_fR.ping();
        leg_fL.ping();
        leg_rL.ping();
        leg_rR.ping();
    
        leg_fR.enablePositionMode();
        leg_fL.enablePositionMode();
        leg_rL.enablePositionMode();
        leg_rR.enablePositionMode();
   
        Serial.println("Robot configuration finished...");
    }

    void setSpeed(double percentage)
    {
        leg_fR.setSpeed(percentage);
        leg_fL.setSpeed(percentage);
        leg_rL.setSpeed(percentage);
        leg_rR.setSpeed(percentage); 
    }


    String stop() {
        this->setSpeed(20.0);
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
       leg_rL.moveTo(1, 14);
       leg_fL.moveTo(1, 14);
       leg_rR.moveTo(1, 14);
       leg_fR.moveTo(1, 14);
       
    }

    /** Walks forward. Keep walking till the new command from Jetson arrives...**/
    String walk()
    {
       int wait = 500;
       while(true) {
          delay(wait);
          leg_rL.moveTo(-6, 15);
          leg_fL.moveTo(3, 12);
          leg_rR.moveTo(3, 12);
          leg_fR.moveTo(-6, 15);
          
          delay(wait);
          leg_fL.moveTo(6, 15);
          leg_rR.moveTo(6, 15);
          
          delay(wait);
          leg_fL.moveTo(-6, 15);
          leg_rL.moveTo(3, 12);
          //...
          leg_fR.moveTo(3, 12);
          leg_rR.moveTo(-6, 15);

          delay(wait);
          leg_rL.moveTo(6, 15);
          leg_fR.moveTo(6, 15);
       }
       return ""; 

      
//      //set velocity before walk sequence...
//      this->setSpeed(70.0);
//      int wait = 350;
//      String nextCommand = CMD_EMPTY;
//      while(nextCommand == CMD_EMPTY) 
//      {
//          delay(wait);
//          // to implement ...
//          
//          nextCommand = getNextCommandFromJetson();
//      }
//      return nextCommand;
    }
    
};

/** Create robot object **/
QuadrupedRobot robot = QuadrupedRobot();


void setup() {
  Serial.begin(115200, SERIAL_8N1);                             // For Uno, Nano, Mini, and Mega, use UART port of DYNAMIXEL Shield to debug.
  shield.begin(57600);                                    // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  shield.setPortProtocolVersion(DXL_PROTOCOL_VERSION);    // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  delay(2000);
  robot.configure();
}


String nextCmd = CMD_STOP;

void loop() 
{
   robot.setSpeed(20.0);

   robot.homePosition();
   delay(2000);
   
   robot.walk();
  
//   robot.homePosition();

//    while(true){
//       delay(2000);
//       robot.moveLegTo(4.0, 16.0);
//       delay(2000);
//       robot.moveLegTo(6.0, 20.0);
//       delay(2000);
//       robot.moveLegTo(1.0, 18.0);
//    }
    
//  if(!nextCmd.equals(CMD_EMPTY)) 
//  {
//      Serial.println("Command received from master: ");
//      Serial.println(nextCmd);
//      if(CMD_STOP.equals(nextCmd))
//      {
//          nextCmd = quadrupedRobot.stop();
//      }
//      if(CMD_WALK_FORWARD.equals(nextCmd))
//      {
//          nextCmd = quadrupedRobot.walk();
//      }
//  }

}
