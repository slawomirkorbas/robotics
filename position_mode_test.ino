/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

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
      int range = 70, rangeCalf = 100;
   
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
  
     void turnOff() {
        shield.torqueOff(dx1);
        shield.torqueOff(dx2);
        shield.torqueOff(dx3);
        shield.setOperatingMode(dx1, OP_POSITION);
        shield.setOperatingMode(dx2, OP_POSITION);
        shield.setOperatingMode(dx3, OP_POSITION);
     }
  
     void turnOn() {
        shield.torqueOn(dx1);
        shield.torqueOn(dx2);
        shield.torqueOn(dx3);
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

     void calfUp() {
        shield.setGoalPosition(dx3, home3 + (inv3 ? -rangeCalf : rangeCalf));
     }

     void calfDown() {
        shield.setGoalPosition(dx3, home3 - (inv3 ? -rangeCalf : rangeCalf));
     }

     void tighBack() {
        shield.setGoalPosition(dx2, home2 + (inv2 ? -range : range));
     }

     void tighForward() {
        shield.setGoalPosition(dx2, home2 - (inv2 ? -range : range));
     }

     void hipInside(int pos) {
        shield.setGoalPosition(dx1, home1 + (inv1 ? -pos : pos));
     }

     void hipOutside(int pos) {
        shield.setGoalPosition(dx1, home1 + (inv1 ? -pos : pos));
     }
     
};

//Front right
Leg leg_fR = Leg(10, 11, 12, 2032-20, 1610-200, 915-30); 
Leg leg_fL = Leg(20, 21, 22, 2044-20, 2480+200, 3234);
Leg leg_rL = Leg(41, 40, 42, 2046-10, 2380+250, 1240);
Leg leg_rR = Leg(31, 30, 32, 2038-10, 1712-250, 2884-35);



void setup() {
  // put your setup code here, to run once:
  
  // For Uno, Nano, Mini, and Mega, use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);

  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  shield.begin(57600);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  shield.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  
    leg_fR.ping();
    leg_fL.ping();
    leg_rL.ping();
    leg_rR.ping();

    leg_fR.turnOff();
    leg_fL.turnOff();
    leg_rL.turnOff();
    leg_rR.turnOff();

    leg_fR.turnOn();
    leg_fL.turnOn();
    leg_rL.turnOn();
    leg_rR.turnOn();

    leg_fR.configureInversion(false,true, false);
    leg_fL.configureInversion(false, false,true);
    leg_rL.configureInversion(false,false, true);
    leg_rR.configureInversion(false,true, false);
    
}



void loop() {
  // put your main code here, to run repeatedly:


    DEBUG_SERIAL.print("leg_fR: ");
    leg_fR.printPosition();
    DEBUG_SERIAL.print("leg_fL: ");
    leg_fL.printPosition();
    DEBUG_SERIAL.print("leg_r1: ");
    leg_rL.printPosition();
    DEBUG_SERIAL.print("leg_r2: ");
    leg_rR.printPosition();

    delay(1000);
    leg_fR.setHome();
    leg_fL.setHome();
    leg_rL.setHome();
    leg_rR.setHome();
    delay(2000);


    leg_fR.hipInside(80);
    leg_fL.hipInside(80);
    leg_rL.hipInside(80);
    leg_rR.hipInside(80);  
    
    // walk...
    int wait = 300;
    while(false) 
    {
        delay(wait);
        leg_rR.calfDown();
        leg_fL.calfDown();
        leg_rR.tighBack();
        leg_fL.tighBack();
        
        leg_fR.calfUp();
        leg_rL.calfUp();
        leg_rL.tighForward();
        leg_fR.tighForward();
        
        delay(wait);
        leg_fR.calfDown();
        leg_rL.calfDown();
        leg_fR.tighBack();
        leg_rL.tighBack(); 
        
        leg_rR.calfUp();
        leg_fL.calfUp();
        leg_rR.tighForward();
        leg_fL.tighForward();

    }
}
