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

const uint8_t DXL_ID = 10;
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

     void setVelocity(double percentage) {
        shield.setOperatingMode(dx1, OP_VELOCITY);
        shield.setOperatingMode(dx2, OP_VELOCITY);
        shield.setOperatingMode(dx1, OP_VELOCITY);
        shield.setGoalVelocity(dx1, percentage, UNIT_PERCENT);
        shield.setGoalVelocity(dx2, percentage, UNIT_PERCENT);
        shield.setGoalVelocity(dx3, percentage, UNIT_PERCENT);
        shield.setOperatingMode(dx1, OP_POSITION);
        shield.setOperatingMode(dx2, OP_POSITION);
        shield.setOperatingMode(dx3, OP_POSITION);
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

class RobotDog {
  public:
    Leg leg_fR = Leg(10, 11, 12, 2032-50, 1610-200, 885    ); 
    Leg leg_fL = Leg(20, 21, 22, 2044-50, 2480+200, 3234   );
    Leg leg_rL = Leg(41, 40, 42, 2046-40, 2380+250, 1240 - 200 );
    Leg leg_rR = Leg(31, 30, 32, 2038-40, 1712-250, 2850 + 200  );

    RobotDog()
    {
      
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

    void setVelocity(double percentage)
    {
        leg_fR.setVelocity(percentage);
        leg_fL.setVelocity(percentage);
        leg_rL.setVelocity(percentage);
        leg_rR.setVelocity(percentage); 
    }


    void homePosition()
    {
        leg_fR.setHome();
        leg_fL.setHome();
        leg_rL.setHome();
        leg_rR.setHome();
    }

  /** Some fancy moves for warm up **/
   void warmUp() 
   {
//        delay(2000);
//        leg_fR.calfUp(250);
//        leg_fL.calfUp(250);
//        leg_rL.calfUp(250);
//        leg_rR.calfUp(250);
//        leg_fR.tighBack(200);
//        leg_fL.tighBack(200);
//        leg_rL.tighBack(200);
//        leg_rR.tighBack(200);
//        delay(1000);
//        
//        int rep = 4;
//        while(0 < rep--)
//        {
//            delay(500);
//            leg_fR.hipInside(80);
//            leg_rR.hipInside(80);
//            leg_fL.hipOutside(170);
//            leg_rL.hipOutside(170);
//            delay(500);  
//            leg_fR.hipOutside(170);
//            leg_rR.hipOutside(170);
//            leg_fL.hipInside(80);
//            leg_rL.hipInside(80);
//        }
//
//        this->homePosition();

        int rep = 4;
        while(0 < rep--)
        {
            delay(1000);
            leg_fR.calfDown(300);
            leg_fL.calfDown(300);
            leg_fR.tighForward(250);
            leg_fL.tighForward(250);
            leg_rR.calfUp(350);
            leg_rL.calfUp(350);
            leg_rR.tighBack(300);
            leg_rL.tighBack(300);
            delay(1000);  
            this->homePosition();
        }
   }

  /**
   * Walking sequence
   */
    void walk()
    {
      int wait = 350;
      while(true) 
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
      }
    }

    void turn(int angle)
    {
         this->homePosition();
         delay(2000);       
         leg_fL.calfDown(100);
         leg_rR.calfDown(100);
         
         while(false) 
         {
           delay(200);
           leg_fL.calfDown(100);
           leg_rR.calfDown(100);
           leg_fR.hipOutside(100); 
           
           delay(200); 
//           leg_fL.calfDown(100);
//           leg_fR.hipOutside(80); 
//           
//           leg_fL.hipOutside(40);
//           leg_rR.hipInside(80);
//           leg_rL.hipOutside(80);
//          
//           leg_fR.hipInside(40);
//           leg_fL.hipOutside(40);
//           leg_rR.hipInside(80);
//           leg_rL.hipOutside(80);
         }
         //this->homePosition();
    }
    
};

/** Create robot object **/
RobotDog dog = RobotDog();

void setup() {
  DEBUG_SERIAL.begin(115200);                             // For Uno, Nano, Mini, and Mega, use UART port of DYNAMIXEL Shield to debug.
  shield.begin(57600);                                    // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  shield.setPortProtocolVersion(DXL_PROTOCOL_VERSION);    // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dog.configure();
}



void loop() 
{
    //dog.setVelocity(20.0);
    dog.homePosition();
//    delay(2000);
//    //DEBUG_SERIAL.println(shield.getPresentVelocity(22, UNIT_PERCENT));
//    //DEBUG_SERIAL.println(shield.setOperatingMode(22, OP_VELOCITY));
//    shield.setGoalVelocity(22, 20.0, UNIT_PERCENT);
//    delay(2000);
//    dog.leg_fL.setVelocity(20.0);
//    dog.leg_fL.calfDown(200);
//    delay(2000);
//    delay(2000);
    
    // test turn 
    //dog.turn(5);
    
    //dog.warmUp();

    delay(2000);
    dog.walk();

}
