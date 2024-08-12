#include <Arduino.h>
#include <Wire.h>
#include <vl53l7cx_class.h>

#ifdef ARDUINO_SAM_DUE
  #define DEV_I2C Wire1
#else
  #define DEV_I2C Wire
#endif
#define SerialPort Serial

#define LPN_PIN A3
#define I2C_RST_PIN A1
TwoWire Wire2(D12, A6);  // Create an instance for the second I2C bus

void send_data_packet(VL53L7CX_ResultsData *Result_top, VL53L7CX_ResultsData *Result_bottom);

// Components.
VL53L7CX sensor_vl53l7cx_1(&Wire, LPN_PIN, I2C_RST_PIN); // Sensor on Wire (I2C bus 1)
VL53L7CX sensor_vl53l7cx_2(&Wire2, LPN_PIN, I2C_RST_PIN); // Sensor on Wire2 (I2C bus 2)

uint8_t res = VL53L7CX_RESOLUTION_8X8;

/* Setup ---------------------------------------------------------------------*/
void setup()
{
  // Initialize serial for output.
  SerialPort.begin(2000000);  // Match the ROS node baud rate

  // Initialize I2C buses.
  Wire.begin();
  Wire2.begin();

  // Configure VL53L7CX components.
  sensor_vl53l7cx_1.begin();
  sensor_vl53l7cx_2.begin();

  sensor_vl53l7cx_1.init_sensor();
  sensor_vl53l7cx_2.init_sensor();

  // Set resolution to 8x8
  sensor_vl53l7cx_1.vl53l7cx_set_resolution(res);
  sensor_vl53l7cx_2.vl53l7cx_set_resolution(res);

  // Set reading speed
  sensor_vl53l7cx_1.vl53l7cx_set_ranging_frequency_hz(15);
  sensor_vl53l7cx_2.vl53l7cx_set_ranging_frequency_hz(15);


  // Start Measurements
  sensor_vl53l7cx_1.vl53l7cx_start_ranging();
  sensor_vl53l7cx_2.vl53l7cx_start_ranging();
  
}

void loop()
{
  VL53L7CX_ResultsData Results1, Results2;
  uint8_t NewDataReady1 = 0, NewDataReady2 = 0;
  uint8_t status1, status2;

  // Check data readiness for sensor 1
  do {
    status1 = sensor_vl53l7cx_1.vl53l7cx_check_data_ready(&NewDataReady1);
  } while (!NewDataReady1);

  if ((!status1) && (NewDataReady1 != 0)) {
    status1 = sensor_vl53l7cx_1.vl53l7cx_get_ranging_data(&Results1);
  }

  // Check data readiness for sensor 2
  do {
    status2 = sensor_vl53l7cx_2.vl53l7cx_check_data_ready(&NewDataReady2);
  } while (!NewDataReady2);

  if ((!status2) && (NewDataReady2 != 0)) {
    status2 = sensor_vl53l7cx_2.vl53l7cx_get_ranging_data(&Results2);
  }

  if ((!status1) && (NewDataReady1 != 0) && (!status2) && (NewDataReady2 != 0)) {
    send_data_packet(&Results1, &Results2);
  }

  delay(1);
}

void send_data_packet(VL53L7CX_ResultsData *Result_top, VL53L7CX_ResultsData *Result_bottom)
{
  for (uint8_t i = 0; i < 64; i++) { // 64 zones for 8x8 resolution
    uint16_t distance_top = Result_top->distance_mm[i];
    uint16_t distance_bottom = Result_bottom->distance_mm[i];
    
    // Send data packet for top sensor
    SerialPort.write(0x7E); // Starting bit
    SerialPort.write(1); // Sensor ID for top sensor
    SerialPort.write(i); // Zone
    SerialPort.write((uint8_t*)&distance_top, sizeof(distance_top)); // Distance
    SerialPort.write(0x7F); // Ending bit
    delay(1); // Short delay to see data packet clearly

    // Send data packet for bottom sensor
    SerialPort.write(0x7E); // Starting bit
    SerialPort.write(2); // Sensor ID for bottom sensor
    SerialPort.write(i); // Zone
    SerialPort.write((uint8_t*)&distance_bottom, sizeof(distance_bottom)); // Distance
    SerialPort.write(0x7F); // Ending bit
    delay(1); // Short delay to see data packet clearly
  }
}
