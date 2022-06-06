static const char *MAIN_TAG = "[main]";
static const char *SEND_TAG = "[sender]";
static const char *TIMER_TAG = "[one-shot timer]";
static const char *BUTTON_TAG = "[button]";
static const char *JOY_TAG = "[joystick]";
static const char *MPU_TAG = "[mpu6050]";

/* Debug flag */
#define BLE_OPEN 0
#define MPU_OPEN 0
#define BTN_OPEN 1
#define JOY_OPEN 0



#define LED_GPIO        GPIO_NUM_2
#define BUTTON_GPIO_X   GPIO_NUM_19
#define BUTTON_GPIO_Y   GPIO_NUM_18
#define BUTTON_GPIO_A   GPIO_NUM_5
#define BUTTON_GPIO_B   GPIO_NUM_17
#define BUTTON_GPIO_L   GPIO_NUM_34
#define BUTTON_GPIO_ZL  GPIO_NUM_39
#define BUTTON_GPIO_R   GPIO_NUM_23
#define BUTTON_GPIO_ZR  GPIO_NUM_36
#define BUTTON_GPIO_DU  GPIO_NUM_32
#define BUTTON_GPIO_DD  GPIO_NUM_25
#define BUTTON_GPIO_DL  GPIO_NUM_26
#define BUTTON_GPIO_DR  GPIO_NUM_27

// Define adc joystick gpio number
#define LJOY_CHANNEL_X  ADC2_CHANNEL_5
#define LJOY_CHANNEL_Y  ADC2_CHANNEL_4
#define LJOY_GPIO_X     ADC2_CHANNEL_5_GPIO_NUM
#define LJOY_GPIO_Y     ADC2_CHANNEL_4_GPIO_NUM
#define LJOY_GPIO_BTN   GPIO_NUM_14
#define RJOY_CHANNEL_X  ADC2_CHANNEL_2
#define RJOY_CHANNEL_Y  ADC2_CHANNEL_3
#define RJOY_GPIO_X     ADC2_CHANNEL_2_GPIO_NUM
#define RJOY_GPIO_Y     ADC2_CHANNEL_3_GPIO_NUM
#define RJOY_GPIO_BTN   GPIO_NUM_16


// MPU
#define PIN_SDA         GPIO_NUM_21
#define PIN_SCL         GPIO_NUM_22
#define I2C_ADDRESS 0x68 // I2C address of MPU6050

#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_PWR_MGMT_1   0x6B
/*
 * The following registers contain the primary data we are interested in
 * 0x3B MPU6050_ACCEL_XOUT_H
 * 0x3C MPU6050_ACCEL_XOUT_L
 * 0x3D MPU6050_ACCEL_YOUT_H
 * 0x3E MPU6050_ACCEL_YOUT_L
 * 0x3F MPU6050_ACCEL_ZOUT_H
 * 0x50 MPU6050_ACCEL_ZOUT_L
 * 0x41 MPU6050_TEMP_OUT_H
 * 0x42 MPU6050_TEMP_OUT_L
 * 0x43 MPU6050_GYRO_XOUT_H
 * 0x44 MPU6050_GYRO_XOUT_L
 * 0x45 MPU6050_GYRO_YOUT_H
 * 0x46 MPU6050_GYRO_YOUT_L
 * 0x47 MPU6050_GYRO_ZOUT_H
 * 0x48 MPU6050_GYRO_ZOUT_L
 */

#define GPIO_INPUT_PIN_SEL ((1ULL << BUTTON_GPIO_X) | (1ULL << BUTTON_GPIO_Y) | (1ULL << BUTTON_GPIO_A) | (1ULL << BUTTON_GPIO_B) \
                        | (1ULL << BUTTON_GPIO_R) | (1ULL << BUTTON_GPIO_L) | (1ULL << BUTTON_GPIO_ZL) | (1ULL << BUTTON_GPIO_ZR) \
                        | (1ULL << BUTTON_GPIO_DU) | (1ULL << BUTTON_GPIO_DD) | (1ULL << BUTTON_GPIO_DL) | (1ULL << BUTTON_GPIO_DR) \
                        | (1ULL << LJOY_GPIO_BTN) | (1ULL << RJOY_GPIO_BTN))


#define ESP_INTR_FLAG_DEFAULT 0
#define FADE_TIME 30000000

#define LEVEL_HIGH 1
#define LEVEL_LOW 0

// How many GPIO port avaliable
#define GPIO_AVALIABLE 64

// Count down button event maximum value
#define CNT_NUM 50

// Joystick max/min value
#define JOY_MAX 32767

// mpu read value struct
struct ring_buf {
    float *buf;
    uint8_t size;
    uint8_t front;
    uint8_t rear;
};