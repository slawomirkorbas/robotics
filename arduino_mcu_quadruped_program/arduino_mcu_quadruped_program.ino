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


const float DXL_PROTOCOL_VERSION = 2.0;

DynamixelShield shield;

//This namespace is required to use Control table item names
using namespace ControlTableItem;

class Leg
{
   private:
      int dx1, dx2, dx3, home1, home2, home3;
      boolean inv1, inv2, inv3;

   public:
     Leg(int s1, int s2, int s3, int h1, int h2, int h3) {
         dx1 = s1;
         dx2 = s2;
         dx3 = s3;
         home1 = h1;
         home2 = h2;
         home3 = h3;
     }

     void configureInversion(boolean i01, boolean i02, boolean i03) {
        inv1 = i01;
        inv2 = i02;
        inv3 = i03;
     }
  
     void enablePositionMode() {
        shield.torqueOff(dx1);
        shield.torqueOff(dx2);
        shield.torqueOff(dx3);
        shield.setOperatingMode(dx1, OP_POSITION);
        shield.setOperatingMode(dx2, OP_POSITION);
        shield.setOperatingMode(dx3, OP_POSITION);
        shield.torqueOn(dx1);
        shield.torqueOn(dx2);
        shield.torqueOn(dx3);
     }

     /** 
      *  Velocity can be in range: 0 - 1023. Before sending it to Dynamixel's control table PROFILE_VELOCITY it has to be multiplicated by 0.229 rpm - what is the unit for this value.
      */
     boolean setSpeed(float speedPct) {
          double maxDynamixelSpeed = 1023 * 0.229; //RPM
          uint32_t newSpeedRpm = speedPct * maxDynamixelSpeed / 100;
          uint32_t writeTimeout = 100; // ms
          shield.writeControlTableItem(PROFILE_VELOCITY, dx1, newSpeedRpm, writeTimeout);
          shield.writeControlTableItem(PROFILE_VELOCITY, dx2, newSpeedRpm, writeTimeout);
          shield.writeControlTableItem(PROFILE_VELOCITY, dx3, newSpeedRpm, writeTimeout);
     }
  
     void setHome() {
        shield.setGoalPosition(dx1, home1);
        shield.setGoalPosition(dx2, home2);
        shield.setGoalPosition(dx3, home3);
     }

     void ping() {
        shield.ping(dx1);
        shield.ping(dx2);
        shield.ping(dx3);
     }

     void printPosition() {
          DEBUG_SERIAL.print(shield.getPresentPosition(dx1));
          DEBUG_SERIAL.print(", ");
          DEBUG_SERIAL.print(shield.getPresentPosition(dx2));
          DEBUG_SERIAL.print(", ");
          DEBUG_SERIAL.print(shield.getPresentPosition(dx3));
          DEBUG_SERIAL.println(" ");
     }

     void calfUp(int pos) {
        shield.setGoalPosition(dx3, home3 + (inv3 ? -pos : pos));
     }

     void calfDown(int pos) {
        shield.setGoalPosition(dx3, home3 - (inv3 ? -pos : pos));
     }

     void tighBack(int pos) {
        shield.setGoalPosition(dx2, home2 + (inv2 ? -pos : pos));
     }

     void tighForward(int pos) {
        shield.setGoalPosition(dx2, home2 - (inv2 ? -pos : pos));
     }

     void hipInside(int pos ) {
        shield.setGoalPosition(dx1, home1 + (inv1 ? -pos : pos));
     }

     void hipOutside(int pos) {
        shield.setGoalPosition(dx1, home1 - (inv1 ? -pos : pos));
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
    Leg leg_fR = Leg(10, 11, 12, 2032-20, 1610-200, 885    ); 
    Leg leg_fL = Leg(20, 21, 22, 2044-20, 2480+200, 3234   );
    Leg leg_rL = Leg(41, 40, 42, 2046-10, 2380+250, 1240 - 200 );
    Leg leg_rR = Leg(31, 30, 32, 2038-10, 1712-250, 2850 + 200  );

    QuadrupedRobot()
    {
      
    }


    /** Check if there is new order from Jetson and reads it from the buffer **/
    String getNextCommandFromJetson() {
        String commandFromJetson = CMD_EMPTY;
        static const char TERMINATOR = '|';
        if (Serial.available() > 0)
        {
          commandFromJetson = Serial.readStringUntil(TERMINATOR);
          // confirm 
          String ackMsg = "ACK: " + commandFromJetson; // String(messageBuffer);
          Serial.print(ackMsg);
          //Serial.flush();
        }
        return commandFromJetson;
    }

  /**
   * Servo smoke test and configuration...
   */
    void configure() 
    {
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
    }

    void setSpeed(double percentage)
    {
        leg_fR.setSpeed(percentage);
        leg_fL.setSpeed(percentage);
        leg_rL.setSpeed(percentage);
        leg_rR.setSpeed(percentage); 
    }


    String homePosition()
    {
        this->setSpeed(20.0);
        String nextCommand = CMD_EMPTY;
        while(nextCommand == CMD_EMPTY) {
          leg_fR.setHome();
          leg_fL.setHome();
          leg_rL.setHome();
          leg_rR.setHome();
          delay(500);
          nextCommand = getNextCommandFromJetson();
        }
        return nextCommand;
    }

   String footWork() {
        // speed up
        this->setSpeed(80.0);
        // footwork
        int rep = 4;
        Leg legs[4] = {leg_fR, leg_fL, leg_rR, leg_rL};
        String nextCommand = CMD_EMPTY;
        while(nextCommand == CMD_EMPTY)
        {  
            for( int i=0; i<4; i++) {
                delay(200);
                legs[i].calfUp(50);
                legs[i].tighBack(50);
                delay(200);
                legs[i].calfDown(50);
                legs[i].tighForward(50);
            }
            nextCommand = getNextCommandFromJetson();
        }
        return nextCommand;
   }


  String swing() {
        // slow down...    
        this->setSpeed(20.0);
        this->homePosition();
        delay(1000);

        String nextCommand = CMD_EMPTY;
        while(nextCommand == CMD_EMPTY)
        {
            delay(500);
            leg_fR.hipInside(70);
            leg_rR.hipInside(70);
            leg_fL.hipOutside(160);
            leg_rL.hipOutside(160);
            delay(500);  
            leg_fR.hipOutside(160);
            leg_rR.hipOutside(160);
            leg_fL.hipInside(70);
            leg_rL.hipInside(70);

            nextCommand = getNextCommandFromJetson();
        }
        return nextCommand;
  }
   

   /** Some fancy moves for warm up **/
   void warmUp() 
   {
        delay(500);
        this->homePosition();

        // sits...
        int rep = 4;
        while(0 < rep--)
        {
            delay(1000);
            leg_fR.calfDown(280);
            leg_fL.calfDown(280);
            leg_fR.tighForward(170);
            leg_fL.tighForward(170);
            leg_rR.calfUp(280);
            leg_rL.calfUp(280);
            leg_rR.tighBack(300);
            leg_rL.tighBack(300);
            delay(1000);
            // give hand
            leg_fR.calfUp(180);
            leg_fR.tighForward(600);
            delay(2000);
            leg_fR.calfDown(280);
            leg_fR.tighForward(170);
            delay(2000);  
            this->homePosition();
        }

        delay(500);
        this->setSpeed(70.0);
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
          leg_rR.calfDown(150);
          leg_fL.calfDown(150);
          leg_rR.tighBack(70);
          leg_fL.tighBack(70);
          
          leg_fR.calfUp(150);
          leg_rL.calfUp(150);
          leg_rL.tighForward(70);
          leg_fR.tighForward(70);
          
          delay(wait);
          leg_fR.calfDown(150);
          leg_rL.calfDown(150);
          leg_fR.tighBack(70);
          leg_rL.tighBack(70); 
          
          leg_rR.calfUp(150);
          leg_fL.calfUp(150);
          leg_rR.tighForward(70);
          leg_fL.tighForward(70);
          
          nextCommand = getNextCommandFromJetson();
      }
      return nextCommand;
    }

    /** Turns the robot. Keep tturning till the new command from Jetson arrives...**/
    String turn(int angle)
    {
         String nextCommand = CMD_EMPTY;
         //slow down a bit 
         delay(500);
         this->setSpeed(50.0);
         delay(500);       
         this->homePosition();
         delay(2000); 

         int rep = 8;
         while(nextCommand == CMD_EMPTY) {
           leg_rR.calfDown(200);
           leg_rR.hipOutside(70);
           leg_rL.hipOutside(120); 
           //    
           leg_fL.calfDown(200);
           leg_fL.hipOutside(70);
           leg_fR.hipOutside(120); 
           delay(200); 
           leg_rL.calfDown(120);
           leg_fR.calfDown(120);
           delay(200); 
           leg_rL.hipInside(20);
           leg_fR.hipInside(20); 
           //     
           leg_rR.hipInside(20);
           leg_rR.calfUp(20);
           leg_fL.hipInside(20);
           leg_fL.calfUp(20);
           delay(200); 

           nextCommand = getNextCommandFromJetson();
         }
         return nextCommand;
    }
    
};

/** Create robot object **/
QuadrupedRobot quadrupedRobot = QuadrupedRobot();


void setup() {
  Serial.begin(115200);                             // For Uno, Nano, Mini, and Mega, use UART port of DYNAMIXEL Shield to debug.
//  while (!Serial) {
//    ; // wait for serial port to connect.
//  }
  shield.begin(57600);                                    // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  shield.setPortProtocolVersion(DXL_PROTOCOL_VERSION);    // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  quadrupedRobot.configure();
}


String nextCmd = CMD_STOP;

void loop() 
{
  if(nextCmd != CMD_EMPTY) 
  {
      if(CMD_STOP.equals(nextCmd))
      {
          nextCmd = quadrupedRobot.homePosition();
      }
      else if(CMD_WALK_FORWARD.equals(nextCmd))
      {
          nextCmd = quadrupedRobot.walk();
      }
      else if(CMD_FOOTWORK.equals(nextCmd))
      {
          nextCmd = quadrupedRobot.footWork();
      }
      else if(CMD_SWING.equals(nextCmd))
      {
          nextCmd = quadrupedRobot.footWork();
      }
  }

}
