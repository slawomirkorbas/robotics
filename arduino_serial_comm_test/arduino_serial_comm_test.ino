void setup()
{
    Serial.begin(115200);

    while (!Serial) {
    ; // wait for serial port to connect.
  }
}

void loop()
{
    // waiting for command from Jetson...
    char messageBuffer[32];
    if (Serial.available() > 0)
    {
      int size = Serial.readBytesUntil('\n', messageBuffer, 32);
      delay(2000);
      // confirm 
      String ackMsg = "Hello Jetson! This is what I got from you: " + String(messageBuffer);
      Serial.println(ackMsg);
    }
}
