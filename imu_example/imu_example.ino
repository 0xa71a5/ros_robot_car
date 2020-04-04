#include "MPU9250.h"
#include "EEPROM.h"

/* MPU 9250 object */
MPU9250 imu(Wire, 0x68);

/* accelerometer and magnetometer data */
float a, ax, ay, az, h, hx, hy, hz;
/* magnetometer calibration data */
float hxb, hxs, hyb, hys, hzb, hzs;
/* euler angles */
float pitch_rad, roll_rad, yaw_rad, heading_rad;
/* filtered heading */
float filtered_heading_rad;
float window_size = 20;
/* conversion radians to degrees */
const float R2D = 180.0f / PI;
/* MPU 9250 object */

/* EEPROM buffer to mag bias and scale factors */
uint8_t eeprom_buffer[24];
float value;

void setup_calibrate() {
  /* Serial for displaying instructions */
  Serial.begin(2000000);
  while(!Serial) {}
  /* Start communication with IMU */
  imu.begin();
  /* Calibrate magnetometer */
  Serial.print("Calibrating magnetometer, please slowly move in a figure 8 until complete...");
  imu.calibrateMag();
  Serial.println("Done!");
  Serial.print("Saving results to EEPROM...");
  /* Save to EEPROM */
  value = imu.getMagBiasX_uT();
  memcpy(eeprom_buffer, &value, sizeof(value));
  value = imu.getMagBiasY_uT();
  memcpy(eeprom_buffer + 4, &value, sizeof(value));
  value = imu.getMagBiasZ_uT();
  memcpy(eeprom_buffer + 8, &value, sizeof(value));
  value = imu.getMagScaleFactorX();
  memcpy(eeprom_buffer + 12, &value, sizeof(value));
  value = imu.getMagScaleFactorY();
  memcpy(eeprom_buffer + 16, &value, sizeof(value));
  value = imu.getMagScaleFactorZ();
  memcpy(eeprom_buffer + 20, &value, sizeof(value));
  for (unsigned int i = 0; i < sizeof(eeprom_buffer); i++) {
    EEPROM.write(i, eeprom_buffer[i]);
  }
  Serial.println("Done! You may power off your board.");
}

void setup(void)
{
    /* Serial for displaying results */
  Serial.begin(115200);
  while(!Serial) {}
  Serial.println("Start imu");
  /* 
  * Start the sensor, set the bandwidth the 10 Hz, the output data rate to
  * 50 Hz, and enable the data ready interrupt. 
  */
  imu.begin();
  imu.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_10HZ);
  imu.setSrd(19);
  /*
  * Load the magnetometer calibration
  */
  Serial.println("Cali mag\n");
  uint8_t eeprom_buffer[24];
  for (unsigned int i = 0; i < sizeof(eeprom_buffer); i++ ) {
    eeprom_buffer[i] = EEPROM.read(i);
  }
  memcpy(&hxb, eeprom_buffer, sizeof(hxb));
  memcpy(&hyb, eeprom_buffer + 4, sizeof(hyb));
  memcpy(&hzb, eeprom_buffer + 8, sizeof(hzb));
  memcpy(&hxs, eeprom_buffer + 12, sizeof(hxs));
  memcpy(&hys, eeprom_buffer + 16, sizeof(hys));
  memcpy(&hzs, eeprom_buffer + 20, sizeof(hzs));
  imu.setMagCalX(hxb, hxs);
  imu.setMagCalY(hyb, hys);
  imu.setMagCalZ(hzb, hzs);
}

/* Bound angle between 0 and 360 */
float constrainAngle360(float dta) {
  dta = fmod(dta, 2.0 * PI);
  if (dta < 0.0)
    dta += 2.0 * PI;
  return dta;
}

void loop()
{
    imu.readSensor();
    ax = imu.getAccelX_mss();
    ay = imu.getAccelY_mss();
    az = imu.getAccelZ_mss();
    hx = imu.getMagX_uT();
    hy = imu.getMagY_uT();
    hz = imu.getMagZ_uT();
    /* Normalize accelerometer and magnetometer data */
    a = sqrtf(ax * ax + ay * ay + az * az);
    ax /= a;
    ay /= a;
    az /= a;
    h = sqrtf(hx * hx + hy * hy + hz * hz);
    hx /= h;
    hy /= h;
    hz /= h;
    /* Compute euler angles */
    pitch_rad = asinf(ax);
    roll_rad = asinf(-ay / cosf(pitch_rad));
    yaw_rad = atan2f(hz * sinf(roll_rad) - hy * cosf(roll_rad), hx * cosf(pitch_rad) + hy * sinf(pitch_rad) * sinf(roll_rad) + hz * sinf(pitch_rad) * cosf(roll_rad));
    heading_rad = constrainAngle360(yaw_rad);
    /* Filtering heading */
    filtered_heading_rad = (filtered_heading_rad * (window_size - 1.0f) + heading_rad) / window_size;
    /* Display the results */
    Serial.print(pitch_rad * R2D);
    Serial.print("\t");
    Serial.print(roll_rad * R2D);
    Serial.print("\t");
    Serial.print(yaw_rad * R2D);
    Serial.print("\t");
    Serial.print(heading_rad * R2D);
    Serial.print("\t");
    Serial.println(filtered_heading_rad * R2D);
    delay(50);
}
