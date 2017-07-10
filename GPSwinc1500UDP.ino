//-------------------------------------------------------------------------------
//  TinyCircuits GPS TinyShield Logging Example
//  Last updated 7 April 2016
//
//  This example uses logs GPS NMEA sentences to SD card. If it doesn't detect an SD
//  card at startup, it will output data to the serial terminal.
//  With the Telit SE868 V2 module with Glonass support, some messages come through
//  as GN** sentences instead of GP**. These are changed back to GP** before logging
//  so that they don't cause problems with programs like Google Earth.
//  Some GPS modules have been shipped with 4800 baud instead of 9600- try this if
//  you see bad data.
//  The Software Serial library should be modified for a larger buffer- 256 is enough
//  for GGA and RMC sentences at 1Hz. In SoftwareSerial.cpp, the change looks like:
//  #define _SS_MAX_RX_BUFF 256
//
//  Written by Ben Rose for TinyCircuits, http://TinyCircuits.com
//
//-------------------------------------------------------------------------------

//This may need to be set to 4800 baud
const int GPSBaud = 9600;

#include "SoftwareSerial256.h"
#include <SPI.h>
#include <WiFi101.h>
#include <WiFiUdp.h>

// The Arduino pins used by the GPS module
const int GPS_ONOFFPin = A3;
const int GPS_SYSONPin = A2;
const int GPS_RXPin = A1;
const int GPS_TXPin = A0;
const int chipSelect = 10;

// The GPS connection is attached with a software serial port
SoftwareSerial Gps_serial(GPS_RXPin, GPS_TXPin);

// Set which sentences should be enabled on the GPS module
char nmea[] = {'1'/*GPGGA*/, '0'/*GNGLL*/, '0'/*GNGSA*/, '0'/*GPGSV/GLGSV*/, '1'/*GNRMC*/, '0'/*GNVTG*/, '0'/*not supported*/, '0'/*GNGNS*/};

int status = WL_IDLE_STATUS;
char ssid[] = "x"; //  your network SSID (name)
char pass[] = "x";    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;            // your network key Index number (needed only for WEP)

unsigned int localPort = 2390;      // local port to listen on

IPAddress sendIP(192, 168, 1, 4);
unsigned int sendPort = 5001;      // local port to listen on

WiFiUDP Udp;

void setup()
{
  WiFi.setPins(8, 2, -1, -1);
  Gps_serial.begin(GPSBaud);
  Serial.begin(115200);
  // Init the GPS Module to wake mode
  pinMode(GPS_SYSONPin, INPUT);
  digitalWrite(GPS_ONOFFPin, LOW);
  pinMode(GPS_ONOFFPin, OUTPUT);
  delay(100);
  Serial.print("Attempting to wake GPS module.. ");
  digitalWrite( GPS_ONOFFPin, HIGH );
  delay(300);
  while (digitalRead( GPS_SYSONPin ) == LOW )
  {
    // Need to wake the module
    digitalWrite( GPS_ONOFFPin, LOW );
    delay(100);

    digitalWrite( GPS_ONOFFPin, HIGH );
    delay(300);
  }
  Serial.println("done.");
  delay(100);


  char command[] = "$PSRF103,00,00,00,01*xx\r\n";
  for (int i = 0; i < 8; i++) {
    command[10] = i + '0';
    command[16] = nmea[i];
    int c = 1;
    byte checksum = command[c++];
    while (command[c] != '*')
      checksum ^= command[c++];
    command[c + 1] = (checksum >> 4) + (((checksum >> 4) < 10) ? '0' : ('A' - 10));
    command[c + 2] = (checksum & 0xF) + (((checksum & 0xF) < 10) ? '0' : ('A' - 10));
    Gps_serial.print(command);
    delay(20);
  }

  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue:
    while (true);
  }

  // attempt to connect to WiFi network:
  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(5000);
  }
  Serial.println("Connected to wifi");
  printWiFiStatus();

  Serial.println("\nStarting connection to server...");
  // if you get a connection, report back via serial:
  Udp.begin(localPort);

  while (Gps_serial.available())
    Gps_serial.read();
}

void loop() {
  while (Gps_serial.read() != '$') {
    //do other stuff here
  }
  while (Gps_serial.available() < 5);
  Gps_serial.read(); Gps_serial.read(); //skip two characters
  char c = Gps_serial.read();
  //determine sentence type
  if (c == 'R' || c == 'G') {
    c = Gps_serial.read();
    if (c == 'M') {
      logNMEA(1);
    } else if (c == 'G') {
      logNMEA(2);
    }
  }
}


void logNMEA(int type) {
  uint8_t buffer[100];
  buffer[0] = '$';
  buffer[1] = 'G';
  buffer[2] = 'P';
  if (type == 1) {
    buffer[3] = 'R';
    buffer[4] = 'M';
  } else if (type == 2) {
    buffer[3] = 'G';
    buffer[4] = 'G';
  }
  int counter = 5;
  char c = 0;
  while (!Gps_serial.available());
  c = Gps_serial.read();
  while (c != '*') {
    buffer[counter++] = c;
    while (!Gps_serial.available());
    c = Gps_serial.read();
  }
  buffer[counter++] = c;
  while (!Gps_serial.available());
  c = Gps_serial.read();
  buffer[counter++] = c;
  while (!Gps_serial.available());
  c = Gps_serial.read();
  buffer[counter++] = c;
  buffer[counter++] = 0x0D;
  buffer[counter++] = 0x0A;
  buffer[counter] = '\0';


  c = 1;
  byte checksum = buffer[c++];
  while (buffer[c] != '*')
    checksum ^= buffer[c++];
  buffer[c + 1] = (checksum >> 4) + (((checksum >> 4) < 10) ? '0' : ('A' - 10));
  buffer[c + 2] = (checksum & 0xF) + (((checksum & 0xF) < 10) ? '0' : ('A' - 10));

  Serial.println((char *)buffer);

  Udp.beginPacket(sendIP, sendPort);
  Udp.write((char *)buffer);
  Udp.endPacket();

}
void printWiFiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}
