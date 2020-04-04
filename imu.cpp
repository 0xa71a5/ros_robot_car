#include "MPU9250.h"
#include "EEPROM.h"
#include "log.h"

/* MPU 9250 object */
static MPU9250 imu(Wire, 0x68);
static float a, ax, ay, az, h, hx, hy, hz;
static float hxb, hxs, hyb, hys, hzb, hzs;
static float pitch_rad, roll_rad, yaw_rad, heading_rad;
static float filtered_heading_rad;
static float window_size = 20;
const float R2D = 180.0f / PI;

/* EEPROM buffer to mag bias and scale factors */
static uint8_t eeprom_buffer[24];

int imu_initialize(void)
{
    pr_debug("Initialize imu...\n");
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
    pr_debug("Write history calibration value to imu\n");
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
    pr_debug("imu initialization complete\n");
}

/* Bound angle between 0 and 360 */
float constrainAngle360(float dta) {
  dta = fmod(dta, 2.0 * PI);
  if (dta < 0.0)
    dta += 2.0 * PI;
  return dta;
}

int imu_get_yaw(float *yaw)
{
    static uint32_t last_read_time = 0;

    /* We read sensor minimum interval is 10ms */
    if (last_read_time == 0 || millis() - last_read_time > 10) {
        imu.readSensor();
        last_read_time = millis();
    }

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

    *yaw = heading_rad * R2D;
    return 0;
}