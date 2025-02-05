#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is conntec to the Arduino digital pin 4
#define ONE_WIRE_BUS 4

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);

// constants
uint8_t celcius_marker [2] = {0x0F, 0x0F};
uint8_t farenheit_marker [2] = {0xF0, 0xF0};

void setup(void)
{
  // Start Serial3 communication for debugging purposes
  Serial.begin(9600);
  Serial3.begin(9600);
  // Start up the library
  sensors.begin();
}

void loop(void){ 
  uint8_t request;
  if (Serial3.available() && (request = Serial3.read()) == 0xFF) {
    // Call sensors.requestTemperatures() to issue a global temperature and Requests to all devices on the bus
    sensors.requestTemperatures(); 

    float c_temp = sensors.getTempCByIndex(0);
    float f_temp = sensors.getTempFByIndex(0);

    // write the all-zero byte to indicate celcius reading
    Serial3.write(celcius_marker[0]); Serial3.write(celcius_marker[1]);
    // write the celcius reading
    uint32_t c_temp_ui = c_temp * 100;
    uint8_t c_temp_bytes[4] = { (uint8_t) (c_temp_ui >> (8*3)), (uint8_t) (c_temp_ui >> (8*2)), (uint8_t) (c_temp_ui >> (8)), (uint8_t) c_temp_ui };
    Serial3.write(c_temp_bytes[0]); Serial3.write(c_temp_bytes[1]);  Serial3.write(c_temp_bytes[2]); Serial3.write(c_temp_bytes[3]);  

    // write the all-one byte to indicate farenheit reading
    Serial3.write(farenheit_marker[0]); Serial3.write(farenheit_marker[1]);
    // write the farenheit reading
    uint32_t f_temp_ui = f_temp * 100;
    uint8_t f_temp_bytes[4] = { (uint8_t) (f_temp_ui >> (8*3)), (uint8_t) (f_temp_ui >> (8*2)), (uint8_t) (f_temp_ui >> (8)), (uint8_t) f_temp_ui };
    Serial3.write(f_temp_bytes[0]); Serial3.write(f_temp_bytes[1]); Serial3.write(f_temp_bytes[2]); Serial3.write(f_temp_bytes[3]);
  }
}
