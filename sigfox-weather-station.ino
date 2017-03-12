#include <LPS25H.h>
#include <HTS221.h>
#include <Wire.h>
#include <Arduino.h>

#define SIGFOX_FRAME_LENGTH 12
#define INTERVAL 600000
#define DEBUG 1

void setup() {
  Wire.begin();
  // Init UART devices
  if (DEBUG) {
    SerialUSB.begin(115200);
  }
  smeHumidity.begin();
  smePressure.begin();
  
  SigFox.begin(19200);

  initSigfox();
}

void loop() {

  // Read sensor values
  double temperature = smeHumidity.readTemperature();
  double humidity = smeHumidity.readHumidity();
  int pressure = smePressure.readPressure();
  if (DEBUG) {     // Printing on SerialUSB doesn't seem to work for the first loop() call
    SerialUSB.print("\nSensors:");
    SerialUSB.print(" Temperature:");                
    SerialUSB.print(temperature);
    SerialUSB.print(" Humidity:");
    SerialUSB.print(humidity);
    SerialUSB.print(" Pressure:");
    SerialUSB.println(pressure);
  }

  // Pack the sensor values into a binary frame
  char frame[SIGFOX_FRAME_LENGTH];
  fillFrame( temperature, humidity, pressure, frame );

  // Encode binary frame as an hexadecimal string 
  String message = encodeFrameAsString( frame );

  // Send the frame to Sigfox network and blink LED indicate modem answer
  bool result = sendStringToSigfox( message );
  if (result) {
    ledGreenLight(HIGH);
    ledRedLight(LOW);
  } else {
    ledGreenLight(LOW);
    ledRedLight(HIGH);
  }
  delay(1000);
  ledGreenLight(LOW);
  ledRedLight(LOW);

  // Wait before sending next frame
  delay(INTERVAL);
}

void fillFrame( double temperature, double humidity, int pressure, char* frame )
{
  // Temperature is clamped to the [-64.0..63.0] range which integer part can be encoded with 6 bits. Two more bits are used to get quarter degree precision
  char packedTemperature = static_cast<char>( round( clampDouble( temperature, -64.0, 63.0 )*4.0 ) + 64.0 );

  // Humidity being a percentage is in the [0.0..100.0] range which integer part can be encoded with 7 bits. Remains one for half percent precision
  char packedHumidity = static_cast<char>( round( clampDouble( humidity, 0.0, 100.0 )*2.0 ) );

  // Pressure is clamped to the [815..1070] hPa range and rebased to the [0..255] range of a single byte
  char packedPressure = static_cast<char>( clampInt( pressure, 815, 1070 )-815 );   

  if (DEBUG) {    
    SerialUSB.print("Packed: Temperature:");
    SerialUSB.print((int)packedTemperature);
    SerialUSB.print(" Humidity:");
    SerialUSB.print((int)packedHumidity);
    SerialUSB.print(" Pressure:");
    SerialUSB.println((int)packedPressure);

    double unpackedTemperature = (static_cast<double>(packedTemperature) - 64.0) / 4.0;
    double unpackedHumidity = static_cast<double>(packedHumidity) / 2.0;
    int unpackedPressure = static_cast<int>(packedPressure)+815;
    SerialUSB.print("Unpacked: Temperature:");
    SerialUSB.print(unpackedTemperature);
    SerialUSB.print(" Humidity:");
    SerialUSB.print(unpackedHumidity);
    SerialUSB.print(" Pressure:");
    SerialUSB.println(unpackedPressure);
  }

  memset( frame, 0, SIGFOX_FRAME_LENGTH );
  frame[0] = packedTemperature;
  frame[1] = packedHumidity;
  frame[2] = packedPressure;
}

double clampDouble(double value, double min, double max)
{
  if ( value<min )
    return min;
  else if (value>max )
    return max;
  return value;
}

double clampInt(int value, int min, int max)
{
  if ( value<min )
    return min;
  else if (value>max )
    return max;
  return value;
}

void initSigfox()
{
  SigFox.print("+++");
  while (!SigFox.available()){
    delay(100);
  }
  while (SigFox.available()){
    byte serialByte = SigFox.read();
    if (DEBUG){
      SerialUSB.print(serialByte);
    }
  }
  if (DEBUG){
    SerialUSB.println("\n ** Setup OK **");
  }
}

String encodeFrameAsString(const char* frame)
{
  String message = "";
  for(int i=SIGFOX_FRAME_LENGTH-1; i>=0; --i) 
  {
    if (frame[i]<16) 
    {
      message += "0";
    }
    message += String(frame[i], HEX);
  }
  if ( DEBUG){
    SerialUSB.print("Message:");
    SerialUSB.println(message);
  }
  return message;
}

bool sendStringToSigfox(const String& message)
{
  String status = "";
  char output;
  if (DEBUG){
    SerialUSB.print("AT$SF=");
    SerialUSB.println(message);
  }
  SigFox.print("AT$SF=");
  SigFox.print(message);
  SigFox.print("\r");
  while ( !SigFox.available() ) 
  {
    // Sleep for a bit?
  }
  
  while(SigFox.available())
  {
    output = (char)SigFox.read();
    status += output;
    delay(10);
  }
  
  if (DEBUG){
    SerialUSB.print("Status:");
    SerialUSB.println(status);
  }
  
  if (status == "OK\r")
  {
    return true;
  }
  else
  {
    return false;
  }
}
