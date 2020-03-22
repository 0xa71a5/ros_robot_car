#include <Wire.h>
#include <MechaQMC5883.h>
#include <MPU9250.h>
#include "queue.h"
#include "crc.h"
#include "log.h"
#include "system.h"
#include "config.h"

#define L_EN    7
#define L_DIR0  6
#define L_DIR1  5
#define R_EN    8
#define R_DIR0  10
#define R_DIR1  4

#define L_ENC0  3
#define L_ENC1  9
#define R_ENC0  2
#define R_ENC1  11

#define MAX_ANALOG_VALUE 255

volatile unsigned long left_enc_cnt = 0;
volatile unsigned long right_enc_cnt = 0;

float distance_per_cnt = 0.00034328;// m/cnt

void set_left_wheel_forward(void)
{
    digitalWrite(L_DIR0, 1);
    digitalWrite(L_DIR1, 0);
}

void set_left_wheel_backward(void)
{
    digitalWrite(L_DIR0, 0);
    digitalWrite(L_DIR1, 1);
}

void set_right_wheel_forward(void)
{
    digitalWrite(R_DIR0, 0);
    digitalWrite(R_DIR1, 1);
}

void set_right_wheel_backward(void)
{
    digitalWrite(R_DIR0, 1);
    digitalWrite(R_DIR1, 0);
}

void set_left_wheel_speed(int value)
{
    if (value < 0)
        value = 0;
    else if (value > MAX_ANALOG_VALUE)
        value = MAX_ANALOG_VALUE;

    analogWrite(L_EN, value);   
}

void set_right_wheel_speed(int value)
{
    if (value < 0)
        value = 0;
    else if (value > MAX_ANALOG_VALUE)
        value = MAX_ANALOG_VALUE;

    analogWrite(R_EN, value);   
}

void stop_lock_left_wheel(void)
{
    digitalWrite(L_DIR0, 0);
    digitalWrite(L_DIR1, 0);
}


void stop_loose_left_wheel(void)
{
    digitalWrite(L_DIR0, 1);
    digitalWrite(L_DIR1, 1);
}

void stop_lock_right_wheel(void)
{
    digitalWrite(R_DIR0, 0);
    digitalWrite(R_DIR1, 0);
}

void stop_loose_right_wheel(void)
{
    digitalWrite(R_DIR0, 1);
    digitalWrite(R_DIR1, 1);
}


void left_encode_isr(void)
{
    if (digitalRead(L_ENC0) == 0) {
        if (digitalRead(L_ENC1) == 1)
            left_enc_cnt--;
        else
            left_enc_cnt++;
    } else {
        if (digitalRead(L_ENC1) == 0)
            left_enc_cnt--;
        else
            left_enc_cnt++;
    }
}

void right_encode_isr(void)
{
    if (digitalRead(R_ENC0) == 0) {
        if (digitalRead(R_ENC1) == 1)
            right_enc_cnt--;
        else
            right_enc_cnt++;
    } else {
        if (digitalRead(R_ENC1) == 0)
            right_enc_cnt--;
        else
            right_enc_cnt++;
    }
}

MPU9250 IMU(Wire,0x68);



void loop_test() {
    int count = 0;
    int l_enc0_val, l_enc1_val, r_enc0_val, r_enc1_val;
    int interval = 100;
    char output_str[64];
    int x,y,z;
    float angle;

    set_left_wheel_forward();
    set_right_wheel_forward();
    set_left_wheel_speed(0);
    set_right_wheel_speed(0);

    stop_loose_left_wheel();
    stop_loose_right_wheel();
    while (1) {
        //compass.read(&x, &y, &z, &angle);
        Serial.println(angle);
        delay(100);
    }

    while (1) {
        l_enc0_val = digitalRead(L_ENC0);
        l_enc1_val = digitalRead(L_ENC1);
        r_enc0_val = digitalRead(R_ENC0);
        r_enc1_val = digitalRead(R_ENC1);

        sprintf(output_str, "[%-4d] (%d,%d) (%d,%d)\n", count, l_enc0_val,
                        l_enc1_val, r_enc0_val, r_enc1_val);
        Serial.print(output_str);
        count++;
        delay(interval);
    }
}

enum STATE_SEARCH {
    STATE_SEARCH_MAGIC0,
    STATE_READ_LENTH,
    STATE_READ_HEADER,
    STATE_READ_BODY,
    STATE_SEARCH_TAIL
};

#define RX_BUFFER_SIZE 248
#define MAX_MESSAGE_LENGTH 128
#define MAX_BODY_LENGTH 128
#define MAX_RESP_DATA_LENGTH 128
#define COMMAND_MAGIC0 0x5a
#define RESP_MAGIC 0x5a


Queue<uint8_t> rx_buffer(RX_BUFFER_SIZE);

uint8_t command_raw[MAX_BODY_LENGTH] = {0};
uint8_t parser_state = STATE_SEARCH_MAGIC0;
#define SPEED_CMD_TIMEOUT_MS 300
uint32_t speed_check_time = 0;

#define MSG_GET_VERSION         0xf1
#define MSG_GET_BATTERY         0x07
#define MSG_GET_ODOM        0x11
#define MSG_GET_IMU             0x13
#define MSG_SET_VEL_CMD         0x01
#define MSG_GET_IMU       0x13

#define MSG_RESP_IMU      0x14
#define MSG_RESP_VERSION            0xf2
#define MSG_RESP_BATTERY            0x08
#define MSG_RESP_ODOM           0x0a
#define MSG_RESP_IMU                0x14

#define VERSION_INFO_LENGTH 6
#define IMU_INFO_LENGTH   32
#define ODOM_INFO_LENGTH 6
#define BATTER_STATE_LENGTH 4

//   MAGIC0   DATA_SIZE   header0   header1    data0 data1 ... dataN    CRC
//     0x5a   total_len     ?      resp_type   |---- data size -----|
uint8_t *send_buffer_init(uint8_t *send_buffer, uint8_t resp_type, uint16_t data_size)
{
    if (data_size >= MAX_RESP_DATA_LENGTH) {
        pr_err("send_buffer_init with data_size %u exceeds range\n", data_size);
        return NULL;
    }

    send_buffer[0] = RESP_MAGIC;
    send_buffer[1] = data_size + 5;
    send_buffer[2] = 0;
    send_buffer[3] = resp_type;

    return send_buffer + 4;
}

int robot_get_version(uint8_t *hardware_version_buffer)
{
    if (hardware_version_buffer) {
        hardware_version_buffer[0] = 0x01;
        hardware_version_buffer[1] = 0x02;
        hardware_version_buffer[2] = 0x03;

        hardware_version_buffer[3] = 0xa0;
        hardware_version_buffer[4] = 0xa1;
        hardware_version_buffer[5] = 0xa2;
    } else {
        pr_err("input param null pointer!\n");
    }

    return SUCCESS;
}

void send_buffer_fill_crc(uint8_t *send_buffer)
{
    uint8_t crc_size = send_buffer[1] - 1;
    uint8_t crc;

    if (crc_size >= MAX_RESP_DATA_LENGTH) {
        pr_err("crc_size %u exceeds range!\n", crc_size);
        return;
    }

    crc = crc_calculate(send_buffer, crc_size);
    //pr_debug("fill crc with 0x%x\n", crc);
    send_buffer[crc_size] = crc;
}

void send_response(uint8_t *send_buffer)
{
    uint8_t send_size = send_buffer[1];
    uint8_t i;

    if (send_size >= MAX_RESP_DATA_LENGTH) {
        pr_err("send_size %u exceeds range!\n", send_size);
        return;
    }

    pr_debug("send size=%u, send_buffer=0x%04x\n", send_size, send_buffer);
    pr_debug("resp ===> [");
    for (i = 0; i < send_size; i++) {
        Serial3.write(send_buffer[i]);
        pr_raw(" 0x%x", send_buffer[i]);
    }
    pr_raw("]\n");
}


int robot_get_battery(uint8_t *data_area)
{
    uint16_t voltage = 12014;// 12.014 V
    uint16_t current = 527;  // 0.527  A

    data_area[0] = voltage >> 8;
    data_area[1] = voltage & 0xff;

    data_area[2] = current >> 8;
    data_area[3] = current & 0xff;

    return SUCCESS;
}

int robot_get_odom(uint8_t *data_area)
{
    static uint32_t last_time = 0;
    uint16_t velocity_vx = 100;
    uint16_t velocity_vyaw = 0;
    static uint16_t yaw_z = 0;

    if (millis() - last_time > 10000) {
        last_time = millis();
        yaw_z = 90 - yaw_z;
    }

    data_area[0] = velocity_vx >> 8;
    data_area[1] = velocity_vx & 0xff;

    data_area[2] = yaw_z >> 8;
    data_area[3] = yaw_z & 0xff;

    data_area[4] = velocity_vyaw >> 8;
    data_area[5] = velocity_vyaw & 0xff;

    return SUCCESS;
}

static inline void pack_int16_value(uint8_t *data_area, int16_t origin_val)
{
    data_area[0] = (origin_val >> 8) & 0xff;
    data_area[1] = origin_val & 0xff;
}

static inline void pack_int32_value(uint8_t *data_area, int32_t origin_val)
{
    data_area[0] = (origin_val >> 24) & 0xff;
    data_area[1] = (origin_val >> 16) & 0xff;
    data_area[2] = (origin_val >> 8) & 0xff;
    data_area[3] = origin_val & 0xff;
}

int robot_get_imu(uint8_t *data_area)
{
    int32_t gyro_x = 1;
    int32_t gyro_y = 1;
    int32_t gyro_z = 1;

    int32_t accl_x = 2;
    int32_t accl_y = 2;
    int32_t accl_z = 2;

    int16_t quat_0 = 0x0;
    int16_t quat_1 = 0x0;
    int16_t quat_2 = 0x0;
    int16_t quat_3 = 0x0;

    pack_int32_value(data_area + 0, gyro_x);
    pack_int32_value(data_area + 4, gyro_y);
    pack_int32_value(data_area + 8, gyro_z);

    pack_int32_value(data_area + 12, accl_x);
    pack_int32_value(data_area + 16, accl_y);
    pack_int32_value(data_area + 20, accl_z);

    //TODO: need to implement quat data
    pack_int16_value(data_area + 24, quat_0);
    pack_int16_value(data_area + 26, quat_1);
    pack_int16_value(data_area + 28, quat_2);
    pack_int16_value(data_area + 30, quat_3);

    return SUCCESS;
}

int robot_vel_control(float vx, float vy, float vyaw)
{
    int left_speed, right_speed;

    pr_debug("recv new robot vel control cmd: vx=%.3f vy=%.3f vyaw=%.3f\n",
              vx, vy, vyaw);

    if (vx > 0) {
        set_left_wheel_forward();
        set_right_wheel_forward();
    } else if (vx < 0) {
        set_left_wheel_backward();
        set_right_wheel_backward();
    }

    vx = abs(vx);
    if (vx > 1)
        vx = 1;

    left_speed = vx * 256;
    right_speed = vx * 256;

    pr_debug("Set speed, left=%d right=%d\n", left_speed, right_speed);
    set_left_wheel_speed(left_speed);
    set_right_wheel_speed(right_speed);

    return SUCCESS;
}

int parse_command_loop(uint8_t *command_raw, uint16_t command_size)
{
    int i;
    uint8_t crc;
    uint8_t msg_type;
    uint8_t send_buffer[MAX_RESP_DATA_LENGTH];
    uint16_t send_size = 0;
    uint8_t *data_area;
    uint8_t command_type;

    // command sanitize check
    if (!command_raw) {
        pr_err("command_raw null ptr!\n");
        return -EINVAL;
    }

    if (command_size < 2 || command_size > MAX_BODY_LENGTH) {
        pr_err("command_size %u exceed legal range\n", command_size);
        return -EINVAL;
    }

    msg_type = command_raw[3];
    pr_debug("Receive new command, size:%u, type:0x%02x data = [", command_size, msg_type);
    for (i = 0; i < command_size; i++) {
        pr_raw(" 0x%x", command_raw[i]);
    }

    crc = crc_calculate(command_raw, command_size - 1);
    pr_raw(" ] |===> crc=0x%x\n", crc);

#ifdef DO_CHECK_CRC
    if (crc != command_raw[command_size - 1]) {
        pr_err("crc check failed! raw crc 0x%x != calc crc 0x%x\n",
                command_raw[command_size - 1], crc);
        return -EPIPE;
    }
#endif


    switch (msg_type) {
        case MSG_GET_VERSION:
        TRACE("Handle MSG_GET_VERSION\n");
        data_area = send_buffer_init(send_buffer, MSG_RESP_VERSION, VERSION_INFO_LENGTH);
        if (!data_area) {
            pr_err("Handle MSG_GET_VERSION failed!\n");
            return -EINVAL;
        }

        if (robot_get_version(data_area)) {
            pr_err("robot_get_version failed!\n");
            return -EINVAL;
        }
        break;

        case MSG_GET_BATTERY:
        TRACE("Handle MSG_GET_BATTERY\n");
        data_area = send_buffer_init(send_buffer, MSG_RESP_BATTERY, BATTER_STATE_LENGTH);
        if (!data_area) {
            pr_err("Handle MSG_GET_VERSION failed!\n");
            return -EINVAL;
        }

        if (robot_get_battery(data_area)) {
            pr_err("robot_get_battery failed!\n");
            return -EINVAL;
        }
        break;

        // Handle request of get velocity and yawz
        case MSG_GET_ODOM:
        TRACE("Handle MSG_GET_ODOM\n");
        data_area = send_buffer_init(send_buffer, MSG_RESP_ODOM, ODOM_INFO_LENGTH);
        if (!data_area) {
            pr_err("Handle MSG_GET_ODOM failed!\n");
            return -EINVAL;
        }

        if (robot_get_odom(data_area)) {
            pr_err("robot_get_odom failed!\n");
            return -EINVAL;
        }
        break;

        case MSG_GET_IMU:
        TRACE("Handle MSG_GET_IMU\n");
        data_area = send_buffer_init(send_buffer, MSG_RESP_IMU, IMU_INFO_LENGTH);
        if (!data_area) {
            pr_err("Handle MSG_GET_ODOM failed!\n");
            return -EINVAL;
        }

        if (robot_get_imu(data_area)) {
            pr_err("robot_get_imu failed!\n");
            return -EINVAL;
        }
        break;

        case MSG_SET_VEL_CMD:
        TRACE("Handle MSG_SET_VEL_CMD\n");
        int16_t vx_raw, vy_raw, vyaw_raw;
        float vx, vy, vyaw;

        vx_raw = command_raw[4] << 8;
        vx_raw |= ((int16_t)command_raw[5] & 0xff);
        vx = float(vx_raw) / 1000;

        vy_raw = command_raw[6] << 8;
        vy_raw |= ((int16_t)command_raw[7] & 0xff);
        vy = float(vy_raw) / 1000;

        vyaw_raw = command_raw[8] << 8;
        vyaw_raw |= ((int16_t)command_raw[9] & 0xff);
        vyaw = float(vyaw_raw) / 1000;
        speed_check_time = millis();
        if (robot_vel_control(vx, vy, vyaw)) {
            pr_err("robot_vel_control failed!\n");
        }

        return;


        default:
        pr_err("Can't find this command handler!\n");
        return -EINVAL;
    }

    // Fill crc and send out response
    send_buffer_fill_crc(send_buffer);
    send_response(send_buffer);
    pr_raw("\n");

    return SUCCESS;
}

uint8_t header_count = 0;
uint8_t body_length = 0;
uint8_t body_count = 0;
uint16_t header_raw;
uint8_t byte_read;
uint8_t message_size = 0;

#define TASK_PERIOD_MS 20
#define BODY_OFFSET 4

void setup() {
    Serial.begin(2000000);
    Serial3.begin(115200);
    log_init();
    pinMode(L_EN, OUTPUT);
    pinMode(L_DIR0, OUTPUT);
    pinMode(L_DIR1, OUTPUT);
    pinMode(R_EN, OUTPUT);
    pinMode(R_DIR0, OUTPUT);
    pinMode(R_DIR1, OUTPUT);
    pinMode(L_ENC0, INPUT);
    pinMode(L_ENC1, INPUT);
    pinMode(R_ENC0, INPUT);
    pinMode(R_ENC1, INPUT);
    pinMode(13, OUTPUT);
    digitalWrite(13, 0);

    //if (IMU.begin() < 0) {
    //    pr_err("IMU initialization failed!\n");
    //}

    digitalWrite(L_EN, 0);
    digitalWrite(R_EN, 0);

    attachInterrupt(digitalPinToInterrupt(L_ENC0), left_encode_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(R_ENC0), right_encode_isr, RISING);
    pr_raw("\n=====> Robot software start <=====\n");
}


void loop()
{
    float acc_x, acc_y;
    uint32_t last_check_time;

    // TODO: Here needs to change to Serial. not Serial3
    while (Serial3.available()) {
        rx_buffer.push(Serial3.read());
    }

    //IMU.readSensor();
    //acc_x = IMU.getAccelX_mss();
    //acc_y = IMU.getAccelY_mss();

    last_check_time = millis();

    if (millis() - speed_check_time > SPEED_CMD_TIMEOUT_MS) {
        // set speed to zero
        pr_debug("cmd_vel not recv within %dms, stop moving\n", SPEED_CMD_TIMEOUT_MS);
        robot_vel_control(0, 0, 0);
        speed_check_time = millis();
    }

    while (rx_buffer.count()) {
        if (millis() - last_check_time > TASK_PERIOD_MS) {
            break;
        }

        if (parser_state == STATE_SEARCH_MAGIC0) {
            //TRACE("STATE : STATE_SEARCH_MAGIC0\n");
            byte_read = rx_buffer.pop();
            if (byte_read != COMMAND_MAGIC0) {
                parser_state = STATE_SEARCH_MAGIC0;
            } else {
                command_raw[0] = byte_read;
                parser_state = STATE_READ_LENTH;
            }
        } else if (parser_state == STATE_READ_LENTH) {
            //TRACE("STATE : STATE_READ_LENTH\n");
            message_size = (uint8_t) rx_buffer.pop();

            if (message_size <= MAX_MESSAGE_LENGTH) {
                // TODO: here need to use macro
                body_length = message_size - 4;
                command_raw[1] = message_size;
                header_count = 0;
                parser_state = STATE_READ_HEADER;
            } else {
                pr_err("message_size %u exceeds range\n", message_size);
                parser_state = STATE_SEARCH_MAGIC0;
            }
        } else if (parser_state == STATE_READ_HEADER) {
            //TRACE("STATE : STATE_READ_HEADER\n");

            byte_read = rx_buffer.pop();
            if (header_count == 0) {
                //TRACE("header0 = 0x%x\n", byte_read);

                command_raw[2] = byte_read;
                parser_state = STATE_READ_HEADER;
            } else if (header_count == 1) {
                //TRACE("header1 = 0x%x\n", byte_read);
                body_count = 0;
                command_raw[3] = byte_read;
                parser_state = STATE_READ_BODY;
            } else {
                body_count = 0;
                parser_state = STATE_SEARCH_MAGIC0;
            }

            header_count++;
        } else if (parser_state == STATE_READ_BODY) {
            //TRACE("STATE : STATE_READ_BODY\n");
            byte_read = rx_buffer.pop();

            if (body_count < body_length) {
                command_raw[BODY_OFFSET + body_count] = byte_read;
                parser_state = STATE_READ_BODY;
                body_count++;
            }

            //TRACE("body_count=%u body_length=%u byte_read=0x%x\n",
            //       body_count, body_length, byte_read);

            if (body_count == body_length) {
                parse_command_loop(command_raw, message_size);
                body_count = 0;
                parser_state = STATE_SEARCH_MAGIC0;
            }
        }
    }
}
