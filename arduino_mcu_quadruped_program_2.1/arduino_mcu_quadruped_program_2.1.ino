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
    float homePos;

    DynamixelServo(int servoId, float h) {
      this->id = servoId;
      this->homePos = h;
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


    void setPositionMode(int mode) {
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

     void printPosition() {
          DEBUG_SERIAL.print("Positon of servo ");
          DEBUG_SERIAL.print(id);
          DEBUG_SERIAL.print(" is ");
          DEBUG_SERIAL.print(shield.getPresentPosition(id));
          DEBUG_SERIAL.println("");
     }
             

     void homePosition() {
        shield.setGoalPosition(id, homePos, UNIT_DEGREE);
     }

     void setPositionRelative(float offsetDeg) {
       shield.setGoalPosition(id, homePos + offsetDeg, UNIT_DEGREE);
     }
};

/**
 * Class representing robot leg consisnting of 3 servos (joints)
 */
class Leg
{
   private:
      boolean inv1, inv2, inv3;
      DynamixelServo *servos[3];

   public:
     Leg(int s1, int s2, int s3, float h1, float h2, float h3) {
        servos[0] = new DynamixelServo(s1,h1);
        servos[1] = new DynamixelServo(s2,h2);
        servos[2] = new DynamixelServo(s3,h3);
     }

     void configureInversion(boolean i01, boolean i02, boolean i03) {
        inv1 = i01;
        inv2 = i02;
        inv3 = i03;
     }
  
     void enablePositionMode() {
        for(int i=0; i<3; i++) {
          servos[i]->setPositionMode(OP_POSITION);
        }
     }

     boolean setSpeed(float speedPct) {
          for(int i=0; i<3; i++) {
             servos[i]->setSpeed(speedPct); 
          }
     }
  
     void setHome() {
         for(int i=0; i<3; i++) {
            servos[i]->homePosition();
         }
     }

     void ping() {
         for(int i=0; i<3; i++) {
            servos[i]->ping();
            servos[i]->printStatus();
         }
     }

     void printPosition() {
         for(int i=0; i<3; i++) {
            servos[i]->printPosition();
         }
     }

     void calfUp(float deg) {
        servos[2]->setPositionRelative(inv3 ? -deg : deg);
     }

     void calfDown(float deg) {
        servos[2]->setPositionRelative( -1* (inv3 ? -deg : deg));
     }

     void tighBack(float deg) {
        servos[1]->setPositionRelative(inv2 ? -deg : deg);
     }

     void tighForward(float deg) {
        servos[1]->setPositionRelative(-1*(inv2 ? -deg : deg));
     }

     void hipInside(float deg ) {
        servos[0]->setPositionRelative((inv1 ? -deg : deg));
     }

     void hipOutside(float deg) {
       servos[0]->setPositionRelative(-1*(inv1 ? -deg : deg));
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
    Leg leg_fR = Leg(10, 11, 12,  179.0,   120.0,   80.0 ); 
    Leg leg_fL = Leg(20, 21, 22,  180.0,   240.0,   280.0 );
    Leg leg_rL = Leg(41, 40, 55,  180.0,   340.0,   100.0 );
    Leg leg_rR = Leg(31, 30, 32,  180.0,   125.0,   260.0 );

//          leg_fR.configureInversion(false, true,  false);
//      leg_fL.configureInversion(false, false, true);
//      leg_rL.configureInversion(false, false, true);
//      leg_rR.configureInversion(false, true,  false);
//    


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
 
      leg_fR.configureInversion(false, true,  false);
      leg_fL.configureInversion(false, false, true);
      leg_rL.configureInversion(false, false, true);
      leg_rR.configureInversion(false, true,  false);
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
          Serial.println("Home position");
          leg_fR.setHome();
          leg_fL.setHome();
          leg_rL.setHome();
          leg_rR.setHome();
          delay(500);
    }


    /** Walks forward. Keep walking till the new command from Jetson arrives...**/
    String walk()
    {
      //set velocity before walk sequence...
      this->setSpeed(70.0);
      int wait = 350;
      String nextCommand = CMD_EMPTY;
      while(nextCommand == CMD_EMPTY) 
      {
          delay(wait);
          // to implement ...
          
          nextCommand = getNextCommandFromJetson();
      }
      return nextCommand;
    }
    
};

/** Create robot object **/
QuadrupedRobot quadrupedRobot = QuadrupedRobot();


void setup() {
  Serial.begin(115200, SERIAL_8N1);                             // For Uno, Nano, Mini, and Mega, use UART port of DYNAMIXEL Shield to debug.
//  while (!Serial) {
//    ; // wait for serial port to connect.
//  }
  shield.begin(57600);                                    // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  shield.setPortProtocolVersion(DXL_PROTOCOL_VERSION);    // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  delay(2000);
  quadrupedRobot.configure();
}


String nextCmd = CMD_STOP;

void loop() 
{
  if(!nextCmd.equals(CMD_EMPTY)) 
  {
      Serial.println("Command received from master: ");
      Serial.println(nextCmd);
      if(CMD_STOP.equals(nextCmd))
      {
          nextCmd = quadrupedRobot.stop();
      }
      if(CMD_WALK_FORWARD.equals(nextCmd))
      {
          nextCmd = quadrupedRobot.walk();
      }
  }

}
