#include <stdio.h>
#include <stdlib.h>
#include "main.h"
#include "BleGamepad.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_sleep.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "NimBLEDevice.h"
#include "driver/adc.h"
#include "esp_system.h"
#include "driver/i2c.h"
#include "math.h"

#undef ESP_ERROR_CHECK
#define ESP_ERROR_CHECK(x)   do { esp_err_t rc = (x); if (rc != ESP_OK) { ESP_LOGE("err", "esp_err_t = %d", rc); assert(0 && #x);} } while(0);


// Variable declare
uint8_t output_data = 0;
int32_t joy_val[4];
int32_t joy_init[4];
esp_err_t err;


static uint32_t send_period = 50;
int cnt = 0;
int cnt2 = 0;

// Used for count down
volatile int64_t *btn_count;
volatile bool dpad_status[4];

static const adc_bits_width_t width = ADC_WIDTH_BIT_12;


static void timer_callback(void* arg);
static void IRAM_ATTR gpio_isr_handler(void* arg);
static int32_t norm_joy(int32_t raw, int32_t center);


static SemaphoreHandle_t sleep_sem;

BleGamepad bleGamepad;
TaskHandle_t senderHandle;
TaskHandle_t btnHandle;
QueueHandle_t xCastQueue;
struct ring_buf rb;


/* Turn on the default LED in the ESP32 board */
static void configure_led(void)
{
    gpio_pad_select_gpio(LED_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_GPIO, LEVEL_HIGH);
}

static void configure_gpio(void){
    
    gpio_config_t io_conf = {};

    //interrupt of falling edge
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io_conf);


    // io_conf.intr_type = GPIO_INTR_ANYEDGE;
    // io_conf.pin_bit_mask = GPI_INPUT_PIN_SEL;
    // io_conf.mode = GPIO_MODE_INPUT;
    // io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    // io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    // gpio_config(&io_conf);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(BUTTON_GPIO_A, gpio_isr_handler, (void*) BUTTON_GPIO_A);
    gpio_isr_handler_add(BUTTON_GPIO_B, gpio_isr_handler, (void*) BUTTON_GPIO_B);
    gpio_isr_handler_add(BUTTON_GPIO_X, gpio_isr_handler, (void*) BUTTON_GPIO_X);
    gpio_isr_handler_add(BUTTON_GPIO_Y, gpio_isr_handler, (void*) BUTTON_GPIO_Y);
    gpio_isr_handler_add(BUTTON_GPIO_L, gpio_isr_handler, (void*) BUTTON_GPIO_L);
    gpio_isr_handler_add(BUTTON_GPIO_R, gpio_isr_handler, (void*) BUTTON_GPIO_R);
    gpio_isr_handler_add(BUTTON_GPIO_ZL, gpio_isr_handler, (void*) BUTTON_GPIO_ZL);
    gpio_isr_handler_add(BUTTON_GPIO_ZR, gpio_isr_handler, (void*) BUTTON_GPIO_ZR);
    gpio_isr_handler_add(BUTTON_GPIO_DU, gpio_isr_handler, (void*) BUTTON_GPIO_DU);
    gpio_isr_handler_add(BUTTON_GPIO_DD, gpio_isr_handler, (void*) BUTTON_GPIO_DD);
    gpio_isr_handler_add(BUTTON_GPIO_DL, gpio_isr_handler, (void*) BUTTON_GPIO_DL);
    gpio_isr_handler_add(BUTTON_GPIO_DR, gpio_isr_handler, (void*) BUTTON_GPIO_DR);
    gpio_isr_handler_add(LJOY_GPIO_BTN, gpio_isr_handler, (void*) LJOY_GPIO_BTN);
    gpio_isr_handler_add(RJOY_GPIO_BTN, gpio_isr_handler, (void*) RJOY_GPIO_BTN);
    
    vTaskDelay( 2 / portTICK_RATE_MS);
    printf("Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size());

    ESP_LOGI(BUTTON_TAG, "GPIO configure is done!");
}

static void configure_adc(void){   
    // Init adc2 channel
    adc2_config_channel_atten(LJOY_CHANNEL_X, ADC_ATTEN_11db );
    adc2_config_channel_atten(LJOY_CHANNEL_Y, ADC_ATTEN_11db );
    adc2_config_channel_atten(RJOY_CHANNEL_X, ADC_ATTEN_11db );
    adc2_config_channel_atten(RJOY_CHANNEL_Y, ADC_ATTEN_11db );

    vTaskDelay(2 / portTICK_RATE_MS);
}

static void configure_mpu(void) {
    	
    ESP_LOGI(MPU_TAG, ">> mpu6050 task start");
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = PIN_SDA;
	conf.scl_io_num = PIN_SCL;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = 100000;
	ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
	ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));

	i2c_cmd_handle_t cmd;
	vTaskDelay(200 / portTICK_RATE_MS);

	cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, 1));
	i2c_master_write_byte(cmd, MPU6050_ACCEL_XOUT_H, 1);
	ESP_ERROR_CHECK(i2c_master_stop(cmd));
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);

	cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, 1));
	i2c_master_write_byte(cmd, MPU6050_PWR_MGMT_1, 1);
	i2c_master_write_byte(cmd, 0, 1);
	ESP_ERROR_CHECK(i2c_master_stop(cmd));
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);

    ESP_LOGI(MPU_TAG, ">> mpu6050 task configure done");
}


/* Select correspond button event to send*/
static void button_event(uint32_t gpio_num) {
    gpio_num_t port = (gpio_num_t)gpio_num;
    uint8_t btn_addr = 0x0;
    bool dpad = false;

    switch (port)
    {
    case BUTTON_GPIO_X:
        btn_addr = BUTTON_4;
        break;
    case BUTTON_GPIO_Y:
        btn_addr = BUTTON_5;
        break;
    case BUTTON_GPIO_A:
        btn_addr = BUTTON_1;
        break;
    case BUTTON_GPIO_B:
        btn_addr = BUTTON_2;
        break;
    case BUTTON_GPIO_L:
        btn_addr = BUTTON_7;
        break;
    case BUTTON_GPIO_R:
        btn_addr = BUTTON_8;
        break;
    case BUTTON_GPIO_ZL:
        btn_addr = BUTTON_9;
        break;
    case BUTTON_GPIO_ZR:
        btn_addr = BUTTON_10;
        break;
    case LJOY_GPIO_BTN:
        btn_addr = BUTTON_14;
        break;
    case RJOY_GPIO_BTN:
        btn_addr = BUTTON_15;
        break;
    case BUTTON_GPIO_DU:
        if(gpio_get_level((gpio_num_t) gpio_num) == LEVEL_LOW){
            dpad_status[0] = true;
        }
        else{
            dpad_status[0] = false;
        }
        dpad = true;
        break;
    case BUTTON_GPIO_DD:
        if(gpio_get_level((gpio_num_t) gpio_num) == LEVEL_LOW){
            dpad_status[1] = true;
        }
        else{
            dpad_status[1] = false;
        }
        dpad = true;
        break;
    case BUTTON_GPIO_DL:
        if(gpio_get_level((gpio_num_t) gpio_num) == LEVEL_LOW){
            dpad_status[2] = true;
        }
        else{
            dpad_status[2] = false;
        }
        dpad = true;
        break;
    case BUTTON_GPIO_DR:
        if(gpio_get_level((gpio_num_t) gpio_num) == LEVEL_LOW){
            dpad_status[3] = true;
        }
        else{
            dpad_status[3] = false;
        }
        dpad = true;
        break;
    default:
        ESP_LOGE(BUTTON_TAG, " button address didn't exist");
        break;
    }

    if(!dpad) {
        if(gpio_get_level((gpio_num_t) gpio_num) == LEVEL_LOW) {
            if(BLE_OPEN)
                bleGamepad.press(btn_addr);
        
            ESP_LOGI(BUTTON_TAG, " button [%d] pressed", btn_addr);
        }
        else {
            if(BLE_OPEN)
                bleGamepad.release(btn_addr);
        
            ESP_LOGI(BUTTON_TAG, " button [%d] released", btn_addr);
        }
    }  
}

/* Task that resume by interrupt, and will send message to PC for a while */
void vEventSender(void *pvParameters){
    ESP_LOGI(SEND_TAG, "Sender task create!!");

    adc2_get_raw( LJOY_CHANNEL_X, width, &joy_init[0]);
    adc2_get_raw( LJOY_CHANNEL_Y, width, &joy_init[1]);
    adc2_get_raw( RJOY_CHANNEL_X, width, &joy_init[2]);
    adc2_get_raw( RJOY_CHANNEL_Y, width, &joy_init[3]);

    int64_t tStart, tEnd;
    tStart = esp_timer_get_time();
    
    while(1){
        //ESP_LOGI(SEND_TAG, "Send message to PC");
        vTaskDelay(send_period / portTICK_RATE_MS);
        tEnd = esp_timer_get_time();
        
        if(!bleGamepad.isConnected() && BLE_OPEN) {
            ESP_LOGW(SEND_TAG, "Device waits for reconnected...");
            vTaskDelay(1000 / portTICK_RATE_MS);
            continue;
        }
            

        // Read joystick adc value
        adc2_get_raw( LJOY_CHANNEL_X, width, &joy_val[0]);
        adc2_get_raw( LJOY_CHANNEL_Y, width, &joy_val[1]);
        adc2_get_raw( RJOY_CHANNEL_X, width, &joy_val[2]);
        adc2_get_raw( RJOY_CHANNEL_Y, width, &joy_val[3]);
        for(int i = 0; i < 4; i++) {
            joy_val[i] = norm_joy(joy_val[i], joy_init[i]);
        }
        ESP_LOGI(JOY_TAG, "LX: [%d] LY: [%d] RX: [%d] RY: [%d]", joy_val[0], joy_val[1], joy_val[2], joy_val[3] );


        
        // Into light sleep mode
        if(tEnd - tStart > FADE_TIME) {
            gpio_set_level(LED_GPIO, LEVEL_LOW);
            ESP_LOGI(SEND_TAG, "Time is up, into deep sleeping mode.....");
            
            vTaskDelay(50 / portTICK_RATE_MS);
            
            // Config wakeup GPIO pin
            esp_sleep_enable_ext0_wakeup(GPIO_NUM_36, LEVEL_HIGH);
            esp_deep_sleep_start();
            ESP_LOGE(SEND_TAG,"This should never be print");
            
            tStart = tEnd;
        }
        if(BLE_OPEN)
            bleGamepad.sendReport();
    }
}

void vTaskMPU6050(void *pvParameters) {

	uint8_t data[14];

	short accel_x;
	short accel_y;
	short accel_z;
    float angle_x;
    float angle_y;
    i2c_cmd_handle_t cmd;
    
    while(1) {
		// Tell the MPU6050 to position the internal register pointer to register
		cmd = i2c_cmd_link_create();
		ESP_ERROR_CHECK(i2c_master_start(cmd));
		ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, 1));
		ESP_ERROR_CHECK(i2c_master_write_byte(cmd, MPU6050_ACCEL_XOUT_H, 1));
		ESP_ERROR_CHECK(i2c_master_stop(cmd));
		ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 5000/portTICK_RATE_MS));
		i2c_cmd_link_delete(cmd);

		cmd = i2c_cmd_link_create();
		ESP_ERROR_CHECK(i2c_master_start(cmd));
		ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_READ, 1));

		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data,   (i2c_ack_type_t)0));
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data+1, (i2c_ack_type_t)0));
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data+2, (i2c_ack_type_t)0));
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data+3, (i2c_ack_type_t)0));
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data+4, (i2c_ack_type_t)0));
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data+5, (i2c_ack_type_t)1));

		//i2c_master_read(cmd, data, sizeof(data), 1);
		ESP_ERROR_CHECK(i2c_master_stop(cmd));
		ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_RATE_MS));
		i2c_cmd_link_delete(cmd);

		accel_x = (data[0] << 8) | data[1];
		accel_y = (data[2] << 8) | data[3];
		accel_z = (data[4] << 8) | data[5];
		// ESP_LOGI(MPU_TAG, "accel_x: %d, accel_y: %d, accel_z: %d", accel_x, accel_y, accel_z);

        angle_x = (float)atan(accel_x / sqrt(pow((double)accel_y, 2) + pow((double)accel_z, 2)));
        angle_y = (float)atan(accel_y / sqrt(pow((double)accel_x, 2) + pow((double)accel_z, 2)));

        ESP_LOGI(MPU_TAG, "angle x: %f, angle y: %f", angle_x, angle_y);

        
		vTaskDelay(50 / portTICK_RATE_MS);
	}

	vTaskDelete(NULL);
}


void vInterrupt(void *pvParameter) {
    
    uint32_t gpio_num;
    BaseType_t receive_success;
    int64_t tNow;


    ESP_LOGI(BUTTON_TAG, "Start button task. ");


    while(1) {
        
        
        tNow = esp_timer_get_time();

        // Clean up all the queue element
        while(xQueuePeek(xCastQueue, &gpio_num, (TickType_t)0) == pdTRUE) {
            // Do not block if receive no data
            receive_success = xQueueReceive(xCastQueue, &gpio_num, ( TickType_t ) 0 );
            if(receive_success == pdTRUE) {
                ESP_LOGI(BUTTON_TAG, " Receive gpio number %d", gpio_num); 
                if (btn_count[gpio_num] == 0) {
                    // First receive event, record current time
                    btn_count[gpio_num] = esp_timer_get_time();
                    // ESP_LOGI(BUTTON_TAG, " time record : %lld", btn_count[gpio_num]);
                }
                else {
                    // Still in count down process, just ignore
                    continue;
                }
            }
            else {
                // Queue empty
                ESP_LOGW(BUTTON_TAG, "This statement cannot happen");
                break;
            }
        }
        
        // Wait time up to trigger button event
        int rest_cnt = 0;
        for(int i = 0; i < GPIO_AVALIABLE; i++) {
            if((btn_count[i] != 0) && ((tNow - btn_count[i]) >= CNT_NUM)) {
                // Judge event happend and press/release button
                btn_count[i] = 0;
                button_event(i);
            }
            else if(btn_count[i] == 0) {
                rest_cnt++;
            }
        }

        if(rest_cnt == GPIO_AVALIABLE) {
            // ESP_LOGI(BUTTON_TAG, "No event to deal with, suspend");
            xSemaphoreTake(sleep_sem, portMAX_DELAY);
        }
    }
}

extern "C" void app_main(void)
{

    BaseType_t status;

    // Debounce button
    btn_count = (int64_t *)malloc(GPIO_AVALIABLE * sizeof(int64_t));
    for(int i = 0; i < GPIO_AVALIABLE; i++) {
        btn_count[i] = 0;
    }

    // DPAD init
    for(int i = 0; i < 4; i++) {
        dpad_status[i] = false;
    }

    // ring buffer to record movement event
    rb = {
        .buf = (float *)malloc(16 * sizeof(float)),
        .size = 16,
        .front = 0,
        .rear = 0
    };
    
    // Create a one-shot timer to record fade out time
    const esp_timer_create_args_t oneshot_timer_args = {
        .callback = &timer_callback,
        .name = "one-shot-timer"
    };

    esp_timer_handle_t oneshot_timer;
    ESP_ERROR_CHECK(esp_timer_create(&oneshot_timer_args, &oneshot_timer));

    // Button GPIO init
    configure_gpio();
    
    // Joystick adc init
    configure_adc();

    // MPU6050 init
    configure_mpu();

    // Initialize Ble Gamepad
    if(BLE_OPEN) {
        ESP_LOGI(MAIN_TAG, "Start ble gamepad init ");
        NimBLEDevice::init("");
        bleGamepad.begin(16, 1, true, true, true, true, true, true, true, true, false, false, false, false, false);
        bleGamepad.setAutoReport(false);
    }

    // Create semaphore and queue elements
    sleep_sem = xSemaphoreCreateBinary();
    xCastQueue = xQueueCreate(16, sizeof(struct b_event *));

    // Task Create
    if(BLE_OPEN) {
        status = xTaskCreate(vEventSender, "EventSender", 8198, NULL, 3, &senderHandle);
        configASSERT(status);
    }
    if(BTN_OPEN) {
        status = xTaskCreate(vInterrupt, "Interrupt test", 4096, NULL, 3, &btnHandle);
        configASSERT(status);
    }
    if(MPU_OPEN) {
        status = xTaskCreate(vTaskMPU6050, "MPUtask", 8198, NULL, 3, NULL);
        configASSERT(status);
    }

    // Turn on the LED to check if system worked
    configure_led();
    ESP_LOGI(MAIN_TAG, "System Ready!");
}

static int32_t norm_joy(int32_t raw, int32_t center) {
    if(raw > (center + 100) && (raw < center - 100)) {
        return 0;
    }else if(raw < 100) {
        return -JOY_MAX;
    }else if(raw > 4000) {
        return JOY_MAX;
    }
    else if(center < 500) {
        // not connect button
        return -1;
    }
    else {
        return (int32_t)((raw - center) / JOY_MAX);
    }
}

static void timer_callback(void* arg)
{
    uint64_t time_since_boot = esp_timer_get_time();
    ESP_LOGI(TIMER_TAG, "One-shot timer Start, time since boot: %lld us", time_since_boot);
}


static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    static BaseType_t xHigherPriorityTaskWoken;
    
    
    xQueueSendFromISR( xCastQueue, (void *)&gpio_num, &xHigherPriorityTaskWoken);
    xSemaphoreGiveFromISR(sleep_sem, &xHigherPriorityTaskWoken);
    gpio_set_level(LED_GPIO, LEVEL_HIGH);

    if( xHigherPriorityTaskWoken )
    {
        // Actual macro used here is port specific.
        portYIELD_FROM_ISR ();
    }
}