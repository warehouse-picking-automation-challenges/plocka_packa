/*
* Author: Fahad Mirza
* fahad.mirza34@mavs.uta.edu
*
* Modified: 04/21/2015
*
* This code is written as a part of Amazon Picking Challenge, for the Team of
* Plocka Packa, lead by Sven Cremer.
*
* ~Brief: Turn on/off the Relay(i.e. valve) by sending ON/OFF through serial port
*  (COM port for Windows, ttyUSB for Linux). It will reply by "ON/OFF Done". Send
*  DATA to get the pressure sensor data.
*/
  #include <string.h>
  
  const char done[7] = " DONE\n";
  const char error[6] = "Error";
  const char on_cmd[3] = "ON";
  const char off_cmd[4] = "OFF";
  const char data_cmd[5] = "DATA";
  char cmd[10];
  int sensorValue;
  
  
  void setup() 
  {
    Serial.begin(9600);        // Initialize serial port
    pinMode(A4, OUTPUT);       // Make A4 pin as digital out. This used to control Relay
    digitalWrite(A4, LOW); 
    analogReference(EXTERNAL); // Set analog reference (AREF) as external. VCC is connected with AREF pin.
  }
  
  void loop() 
  {
    if (Serial.available() > 0)              // Is data available?
    {
      Serial.readBytesUntil('\n', cmd, 10);  // Read until you encounter with NewLine 
        
      if (strncmp(cmd, on_cmd, 2) == 0)      // Is it a ON command?
      {
        digitalWrite(A4, HIGH);              // Set A4 High
        Serial.print(on_cmd);                // Send Ack
        Serial.print(done);
      }
      else if(strncmp(cmd, off_cmd, 3) == 0) // Is it a OFF command?
      {
        digitalWrite(A4, LOW);
        Serial.print(off_cmd);
        Serial.print(done);
      }
      else if(strncmp(cmd, data_cmd, 4) == 0)// Is it a DATA command?
      {
        sensorValue = analogRead(A7);        // Read sensor data from A7
        Serial.println(sensorValue);         // Print data
        delay(1);
      }
      else
      {
        Serial.println(error);
      }
    }
  }
