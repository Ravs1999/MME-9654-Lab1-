/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */


//Ravneet Rattan, 250956818

/*Code Overview
-Modified code of Dr. Naish's Debounce code used 
-Modified examples of oneshot ADC implementation used 
*/

#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/pulse_cnt.h"
#include "bdc_motor.h"
#include "pid_ctrl.h"
#include "driver/gpio.h"
#include <inttypes.h>
#include "soc/soc_caps.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

static const char *TAG = "lab1";
static const char *TAG2 = "debounce";

// Enable this config,  we will print debug formated string, which in return can be captured and parsed by Serial-Studio
#define SERIAL_STUDIO_DEBUG           CONFIG_SERIAL_STUDIO_DEBUG

//MCPWM Defines 
#define BDC_MCPWM_TIMER_RESOLUTION_HZ 10000000 // 10MHz, 1 tick = 0.1us
#define BDC_MCPWM_FREQ_HZ             25000    // 25KHz PWM
#define BDC_MCPWM_DUTY_TICK_MAX       (BDC_MCPWM_TIMER_RESOLUTION_HZ / BDC_MCPWM_FREQ_HZ) // maximum value we can set for the duty cycle, in ticks
#define BDC_MCPWM_GPIO_A              7
#define BDC_MCPWM_GPIO_B              15

#define BDC_ENCODER_GPIO_A            36
#define BDC_ENCODER_GPIO_B            35
#define BDC_ENCODER_PCNT_HIGH_LIMIT   1000
#define BDC_ENCODER_PCNT_LOW_LIMIT    -1000

#define BDC_PID_LOOP_PERIOD_MS        10   // calculate the motor speed every 10ms
//#define BDC_PID_EXPECT_SPEED          100   // expected motor speed, in the pulses counted by the rotary encoder

// GPIO Defines 
#define LED_GPIO GPIO_NUM_16  //define LED GPIO pin
#define BUTTON_GPIO GPIO_NUM_10 //define button GPIO pin
#define GPIO_BUTTON_PIN_SEL   (1ULL << BUTTON_GPIO)                          // set bit for button GPIO
#define DEBOUNCE_INTERVAL     100                                           // set debounce interval in milliseconds

//#define BLINK_GPIO CONFIG_BLINK_GPIO
static TickType_t next = 0;                                                  // tick count for next allowable button press
static uint32_t number_presses = 0;                                          // counter for number of button presses
static bool pressed = false;                                                 // state variable to indicate "valid" button press
volatile long mapped;                                                        //global variable for speed value in pulses to be fed to pid controller function 

// Interrupt service routine                                                
static void IRAM_ATTR button_pressed_handler(void *arg)    //for better latency, use IRAM_ATTR                          
{                                                                           
   TickType_t now = xTaskGetTickCountFromISR();                              // capture current ticks
   if (now > next) {                                                         // check whether enough time has passed to consider a valid press
      number_presses++;                                                      // increment button press counter
      pressed = true;                                                        // indicate valid button press state
      next = now + DEBOUNCE_INTERVAL / portTICK_PERIOD_MS;                   // set tick count for next allowable button press
   }                                                                        
}                                                                           


// Initialize button GPIO and install ISR                                           
static void button_init(void)                                               
{                                                                           
   gpio_config_t io_conf;                                                    // configuration structure
                                                                            
   io_conf.pin_bit_mask = GPIO_BUTTON_PIN_SEL;                               // select pin for pushbutton
   io_conf.mode = GPIO_MODE_INPUT;                                           // configure as input
   io_conf.pull_up_en = GPIO_PULLUP_ENABLE;                                  // enable internal pullup resistor
   io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;                             // disable internal pulldown resistor
   io_conf.intr_type = GPIO_INTR_NEGEDGE;                                    // trigger interrupt on negative edge 
   ESP_ERROR_CHECK(gpio_config(&io_conf));                                   // configure pushbutton pin
   
   ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1));          // allocate interrupt with lowest priority
   ESP_ERROR_CHECK(gpio_isr_handler_add(BUTTON_GPIO, button_pressed_handler, NULL)); // setup ISR for button 
} 


//Initialize LED GPIO 
static void configure_led(void)
{
    //ESP_LOGI(TAG, "Example configured to blink GPIO LED!");
    gpio_reset_pin(LED_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
}

//basic led blink function
static void blink_led(int led_state)
{
    /* Set the GPIO level according to the state (LOW or HIGH)*/
    gpio_set_level(LED_GPIO, led_state);
}


/*ADC Setup and Function Definitions*/ 

#define EXAMPLE_ADC1_CHAN1 ADC_CHANNEL_3 //ADC1 Channel
#define EXAMPLE_ADC_ATTEN  ADC_ATTEN_DB_11

static int adc_raw[1][10]; //

static bool example_adc_calibration_init(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void example_adc_calibration_deinit(adc_cali_handle_t handle);
long Map(long adc_val, long in_min, long in_max, long out_min, long out_max);


/*MOTOR Object and PID Controller Setup*/
typedef struct {
    bdc_motor_handle_t motor;
    pcnt_unit_handle_t pcnt_encoder;
    pid_ctrl_block_handle_t pid_ctrl;
    int report_pulses;
} motor_control_context_t;

static void pid_loop_cb(void *args) 
{
    static int last_pulse_count = 0;
    motor_control_context_t *ctx = (motor_control_context_t *)args;
    pcnt_unit_handle_t pcnt_unit = ctx->pcnt_encoder;
    pid_ctrl_block_handle_t pid_ctrl = ctx->pid_ctrl;
    bdc_motor_handle_t motor = ctx->motor;

    // get the result from rotary encoder
    int cur_pulse_count = 0;
    pcnt_unit_get_count(pcnt_unit, &cur_pulse_count);
    int real_pulses = cur_pulse_count - last_pulse_count;
    last_pulse_count = cur_pulse_count;
    ctx->report_pulses = real_pulses;

    // calculate the speed error
    float error = mapped - abs(real_pulses); //use scaled pulse value from potentiometer 
    //float error = BDC_PID_EXPECT_SPEED - real_pulses;
    float new_speed = 0;

    // set the new speed
    pid_compute(pid_ctrl, error, &new_speed);
    bdc_motor_set_speed(motor, (uint32_t)new_speed);
}



void app_main(void)
{
    //initialize led and button gpio
    button_init();
    configure_led();
    int blink_Interval=300; //300ms 
    int64_t prevMillis=0; //store last time LED updated
    int led_State=0; 

    //ADC1 Initialization and Instanciation
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    //ADC1 Config
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = EXAMPLE_ADC_ATTEN,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, EXAMPLE_ADC1_CHAN1, &config));

    //ADC1 Calibration Init
    adc_cali_handle_t adc1_cali_handle = NULL;
    bool do_calibration1 = example_adc_calibration_init(ADC_UNIT_1, EXAMPLE_ADC_ATTEN, &adc1_cali_handle);


    //MCPWM Initialization and Instance
    static motor_control_context_t motor_ctrl_ctx = {
        .pcnt_encoder = NULL,
    };

    ESP_LOGI(TAG, "Create DC motor");
    bdc_motor_config_t motor_config = {
        .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
        .pwma_gpio_num = BDC_MCPWM_GPIO_A,
        .pwmb_gpio_num = BDC_MCPWM_GPIO_B,
    };
    bdc_motor_mcpwm_config_t mcpwm_config = {
        .group_id = 0,
        .resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ,
    };
    bdc_motor_handle_t motor = NULL;
    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor_config, &mcpwm_config, &motor));
    motor_ctrl_ctx.motor = motor;

    ESP_LOGI(TAG, "Init pcnt driver to decode rotary signal");
    pcnt_unit_config_t unit_config = {
        .high_limit = BDC_ENCODER_PCNT_HIGH_LIMIT,
        .low_limit = BDC_ENCODER_PCNT_LOW_LIMIT,
        .flags.accum_count = true, // enable counter accumulation
    };
    pcnt_unit_handle_t pcnt_unit = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = BDC_ENCODER_GPIO_A,
        .level_gpio_num = BDC_ENCODER_GPIO_B,
    };
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));
    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = BDC_ENCODER_GPIO_B,
        .level_gpio_num = BDC_ENCODER_GPIO_A,
    };
    pcnt_channel_handle_t pcnt_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, BDC_ENCODER_PCNT_HIGH_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, BDC_ENCODER_PCNT_LOW_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));
    motor_ctrl_ctx.pcnt_encoder = pcnt_unit;

    ESP_LOGI(TAG, "Create PID control block");
    pid_ctrl_parameter_t pid_runtime_param = {
        .kp = 0.6,
        .ki = 0.4,
        .kd = 0.2,
        .cal_type = PID_CAL_TYPE_INCREMENTAL,
        .max_output   = BDC_MCPWM_DUTY_TICK_MAX - 1,
        .min_output   = 0,
        .max_integral = 1000,
        .min_integral = -1000,
    };
    pid_ctrl_block_handle_t pid_ctrl = NULL;
    pid_ctrl_config_t pid_config = {
        .init_param = pid_runtime_param,
    };
    ESP_ERROR_CHECK(pid_new_control_block(&pid_config, &pid_ctrl));
    motor_ctrl_ctx.pid_ctrl = pid_ctrl;

    ESP_LOGI(TAG, "Create a timer to do PID calculation periodically");
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = pid_loop_cb,
        .arg = &motor_ctrl_ctx,
        .name = "pid_loop"
    };
    esp_timer_handle_t pid_loop_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &pid_loop_timer));

    ESP_LOGI(TAG, "Enable motor");
    ESP_ERROR_CHECK(bdc_motor_enable(motor));
    ESP_LOGI(TAG, "Forward motor");
    ESP_ERROR_CHECK(bdc_motor_forward(motor));

    ESP_LOGI(TAG, "Start motor speed loop");
    ESP_ERROR_CHECK(esp_timer_start_periodic(pid_loop_timer, BDC_PID_LOOP_PERIOD_MS * 1000));

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(100));
        if (number_presses % 2 ==0){ //if even button count, motor spins in forward mode
            pressed = false;
            ESP_LOGI(TAG, "Forward motor");
            ESP_ERROR_CHECK(bdc_motor_forward(motor));
            blink_led(1);
        }

        else if(number_presses % 2 !=0){ //if button count odd, motor spins in reverse mode 
            pressed = false; 
            ESP_LOGI(TAG, "Reverse motor");
            ESP_ERROR_CHECK(bdc_motor_reverse(motor));  
            int64_t currentMillis=esp_timer_get_time()/1000;

            if(currentMillis-prevMillis > blink_Interval){
                //save last time LED blinked 
                prevMillis=currentMillis; 
                if (led_State==0) led_State=1;
                else led_State=0;
                blink_led(led_State);
            }
            
        }

        int sum_adc=0;
        for (int i=0;i<20;i++){
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN1, &adc_raw[i]));
        sum_adc+=adc_raw[i]   
        }

        int average_adc=sum_adc/20; //20 elements in adc_raw array 
        //ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN1, &adc_raw[0][1]));
        //ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, EXAMPLE_ADC1_CHAN1, adc_raw[0][1]);
        mapped = Map(average_adc, 0, 4095, 0, 50);
        int error=mapped- abs(motor_ctrl_ctx.report_pulses);
          
        // the following logging format is according to the requirement of serial-studio frame format
        // also see the dashboard config file `serial-studio-dashboard.json` for more information
#if SERIAL_STUDIO_DEBUG
       printf("/*%d,%d*/\r\n", motor_ctrl_ctx.report_pulses,error);
    
#endif

    }
}


//ADC Calibration Function Definitions
static bool example_adc_calibration_init(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

static void example_adc_calibration_deinit(adc_cali_handle_t handle)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Curve Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));
#endif
}


// Convert RAW ADC reading to a scaled pulse value to send to pid control loop
long Map(long adc_val, long in_min, long in_max, long out_min, long out_max)
{
    return (adc_val - in_min)* (out_max - out_min) / (in_max - in_min) + out_min;
}