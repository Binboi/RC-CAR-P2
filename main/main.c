

////////DEVICE 1/////////////////////////
//Modify file
//git add .
//git commit -m "Describe your changes"
//git push -u origin main

////////Device 2/////////////////////////
//git pull

#include <stdio.h>
#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "usb/usb_host.h"

#include "radio.h"
#include "usb_capture.h"

// not all are used, some are for reference like adc pins (e.g: first three)
#define adc_steering_input 18 
#define adc_throttle_input 8
#define adc_braking_input 3
#define servo 4 //
#define motor 5 //
#define IN1 7 //
#define IN2 15 //

int steering_val, throttle_val, brake_val;
static adc_oneshot_unit_handle_t adc1_handle;
static adc_oneshot_unit_handle_t adc2_handle;
uint16_t duty_steering; //
uint16_t duty_throttle; //
uint16_t duty_brake; //

//unused for now
int angle;
float steering_k;
float steering_sens;
float target_speed_left;
float target_speed_right;

void input_init();
void adc_setup();
void pwm_config(); //
void getVal();
void set_steering(); 
void set_throttle();
void setVal_TX_Background(void *pvParameters);
void printVal_Background(void *pvParameters);
void fwd();
void rev();
void brake();

////////////UNUSED FUNCS AND VARS FOR LATER USE////////////////////
int left_setpoint, right_setpoint;
int output_left, output_right;
void diff_calc();

void ackerman();
void PID(int setpoint, int val);
///////////////////////////////////////////////////////////////////

void app_main(){

    //start usb protocol (printing as well)
    usb_capture_start();

    //Initialize inputs
    input_init();

    //Initialize oneshot ADC module
    adc_setup();

    //Initialize PWM for the servo
    pwm_config();

    //Initialize radio protocol
    nrf_init();

    //Start the background car value setting process
    xTaskCreate(setVal_TX_Background, "Live Values", 2048, NULL, 5, NULL);

    //Delay so the follow print task has values to print
    vTaskDelay(10 / portTICK_PERIOD_MS);

    //Start the background printing process (with joystick)
    xTaskCreate(printVal_Background, "Live Values", 2048, NULL, 5, NULL);

}

//background task (setting values, radio transmits)
void setVal_TX_Background(void *pvParameters){

    while(1){

        //gets the adc value (driving inputs)
        getVal();
        //calcualtes duty cycles and sets them to pwm pins
        set_steering();
        set_throttle();

        //transmit through radio


        vTaskDelay(10 / portTICK_PERIOD_MS);

    }

}

//background task (printing values, slower task than above)
void printVal_Background(void *pvParameters){

    printf("Joystick program starting...\n");

    printf("CONFIG:    0x%02X (expect 0x0E for TX)\n",  nrf_read_reg(0x00));
    printf("EN_AA:     0x%02X (expect 0x00)\n",          nrf_read_reg(0x01));
    printf("EN_RXADDR: 0x%02X (expect 0x01)\n",          nrf_read_reg(0x02));
    printf("AW:        0x%02X (expect 0x03)\n",          nrf_read_reg(0x03));
    printf("RETR:      0x%02X (expect 0x00)\n",          nrf_read_reg(0x04));
    printf("RF_CH:     0x%02X (expect 0x7C)\n",          nrf_read_reg(0x05));
    printf("RF_SETUP:  0x%02X (expect 0x26)\n",          nrf_read_reg(0x06));
    printf("STATUS:    0x%02X (expect 0x0E)\n",          nrf_read_reg(0x07));
    printf("RX_PW_P0:  0x%02X (expect 0x04)\n",          nrf_read_reg(0x11));

    vTaskDelay(200 / portTICK_PERIOD_MS);
    
    //hides the cursor
    printf("\033[?25l");

    while(1){

        // print values with updates instead of new lines (\r vs \n)
        printf("Steering: %4d, Throttle: %4d, Braking: %d , Steering Duty Cycle: %4d, Throttle Duty Cycle: %4d     \n", 
            steering_val, throttle_val, brake_val, duty_steering, duty_throttle);
        
        uint8_t data[4];
        //steering duty cycle data
        data[0] = (uint8_t)(duty_steering >> 8) & 0x0F;
        data[1] = (uint8_t)(duty_steering & 0xFF);
        //throttle duty cycle data
        data[2] = (uint8_t)(duty_throttle >> 8) & 0x0F;
        data[3] = (uint8_t)(duty_throttle & 0xFF);

        nrf_transmit(data, 4);

        // wait and check STATUS
        vTaskDelay(1 / portTICK_PERIOD_MS);
        uint8_t status = nrf_read_reg(NRF_STATUS);

        //print the status of the radio transmit
        if (status & 0x20) {

            /*
            printf("TX success | Transmission: Steering Duty Cycle: %4d Throttle Duty Cycle: %4d, Data1: %4d   \r",
                duty_steering, duty_throttle, data[1]);
            */

            printf("TX success | Transmission: Steering Duty Cycle: %4d, Data1: %4d   \r",
                duty_steering, data[1]);

        } 
        
        else if (status & 0x10) {

            printf("TX failed — no Acknowledge   \r");
            //clears flag message
            nrf_write_reg(NRF_STATUS, 0x10);
            nrf_flush_tx();

        }

        //

        //clears TX_DS flag
        nrf_write_reg(NRF_STATUS, 0x20); 

        printf("\033[1A\r");
        fflush(stdout);

        vTaskDelay(150 / portTICK_PERIOD_MS);

    }

}

void input_init(){

    //Reset the button pin to the default state
    gpio_reset_pin(servo);
    gpio_reset_pin(motor);

    //Configure pin 16 as output for IN1
    gpio_set_direction(IN1, GPIO_MODE_OUTPUT);

    //Configure pin 17 as output for IN2
    gpio_set_direction(IN2, GPIO_MODE_OUTPUT);

}

void adc_setup(){

    adc_oneshot_unit_init_cfg_t init_config1 = {

        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,

    };

    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_unit_init_cfg_t init_config2 = {
    .unit_id = ADC_UNIT_2,
    .ulp_mode = ADC_ULP_MODE_DISABLE,

    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config2, &adc2_handle));

    

    //Configure ADC channels
    adc_oneshot_chan_cfg_t config = {

        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };

    //steering adc channel (pin 18)
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, ADC_CHANNEL_7, &config));
    vTaskDelay(5 / portTICK_PERIOD_MS);
    //throttle adc channel (pin 8)
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_7, &config));
    //braking adc channel (pin 8)
    vTaskDelay(5 / portTICK_PERIOD_MS); 
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_2, &config));

}

void pwm_config(){

    // Setup the timer for the servos (slower)
    ledc_timer_config_t servo_timer_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_12_BIT,  // 12-bit = 0-4095
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 50,                         // 50 Hz for servos
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&servo_timer_config);

    // Setup the timer for the motor (faster)
    ledc_timer_config_t motor_timer_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_12_BIT,  // 12-bit = 0-4095
        .timer_num = LEDC_TIMER_1,
        .freq_hz = 5000,                         // 5 kHz for servos
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&motor_timer_config);

    // Setup the PWM channel for the steering
    ledc_channel_config_t channel_config = {
        .gpio_num = servo,
        .speed_mode = LEDC_LOW_SPEED_MODE,  
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&channel_config);

    // Setup the PWM channel for EN1 (on driving motor)
    ledc_channel_config_t channel_config2 = {
        .gpio_num = motor,
        .speed_mode = LEDC_LOW_SPEED_MODE,

        .channel = LEDC_CHANNEL_1,
        .timer_sel = LEDC_TIMER_1,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&channel_config2);

}

void getVal(){

    // Read ADC values for GPIOs 18, 8 and 3
    ESP_ERROR_CHECK(adc_oneshot_read(adc2_handle, ADC_CHANNEL_7, &steering_val));
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_7, &throttle_val));
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_2, &brake_val));


}

void set_steering(){

//Duty cycles from trial and error
//Mid point: 282
//Full clockwise: 80
//Full counter clockwise: 493

    //Rested X values for ADC tested from serial monitor (1820 - 1840 for 3.3v)
    if(steering_val > 2800 &&  steering_val < 2900){

        // Tested value from serial monitor
        duty_steering = 282; //Mid point: ((80 + 493) / 2)    

    }

    else{

        // Map the joystick x value reading to 12 bit pwm range (Trial and error extremes)

        //for 3.3V
        duty_steering = 80 + ((steering_val * (493 - 80)) / 4095);

        //for 5.5V
        //duty_steering = steering_val;

    }

    // Stores the new duty cycle variable in memory
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty_steering);
    // Sets the steering duty cycle to "duty_steering"
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

}

void set_throttle(){

    //Middle of braking potentiometer
    //rest if throtthle is else because braking takes priority
    if(brake_val > 700){

        // More brake, less throttle duty cycle
        duty_throttle = duty_throttle - ((brake_val * duty_throttle) / 4095);

        //makes sure duty_throttle doesnt go negative from above calc
        //converts to signed int so it can check for negatives and make it zero
        int32_t temp = duty_throttle;
        if(temp < 0){ temp = 0; }
        duty_throttle = (uint16_t)temp;

        //if fully pressing brakes, stop motor
        if(brake_val > 3500){

            brake();
            duty_throttle = 0;

        }

    }

    //another braking condition for when no speed is applied
    else if(throttle_val >= 2850 && throttle_val <= 2950){

        duty_throttle = 0;
        brake();

    }

    else if(throttle_val < 2850){

        //Maps 0-2950 to PWM duty cycle scale (0-4095)
        //duty_throttle = ((abs(2900 - throttle_val)) / 2900) * 4905; rewritten to avoid truncation
        //2900 is the more accurate rested y value
        duty_throttle = ((abs(2900 - throttle_val)) * 4095) / 2900;
        rev();

    }

    else if(throttle_val > 2950){

        //Maps 2900 - 4095 to PWM duty cycle scale (0-4095) (0 - 4095)
        //duty_throttle = ((throttle_val - 2900) / (4095 - 2900)) * 4095; rewritten to avoid truncation
        //2900 is the more accurate rested y value
        duty_throttle = ((throttle_val - 2900) * 4095) / (4095 - 2900);
        fwd();

    }

    // Stores the new duty cycle variable in memory
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty_throttle);
    // Sets the throttle PWM duty cycle to "duty_throttle"
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);

}

void fwd(){

    gpio_set_level(IN1, 1);
    gpio_set_level(IN2, 0);

}

void rev(){

    gpio_set_level(IN1, 0);
    gpio_set_level(IN2, 1); 

}

void brake(){

    gpio_set_level(IN1, 0);
    gpio_set_level(IN2, 0);

}

//////////////////////UNUSED FUNCTIONs, IGNOER/////////////////////////////

void diff_calc(){

    if(duty_steering == 80){

        steering_k = -1;
    
    }

    if(duty_steering == 493){

        steering_k = 1;
    
    }

    if(duty_steering == 282){

        steering_k = 0;
        
    }

    if(duty_steering > 282){

        steering_k = (duty_steering - 282) / (493 - 282) ;

    }

    if(duty_steering < 282){

        steering_k =  -((float)(282 - duty_steering)) / (282 - 80);

    }

}

void ackerman(){

    steering_sens = 1.5;  //change depending on duty_throttle vs duty_steering scale
    
    target_speed_left = duty_throttle + (steering_sens * steering_k);
    target_speed_right = duty_throttle - (steering_sens * steering_k);

    if(target_speed_left < 0){

        target_speed_left = 0;

    }

    if(target_speed_right > 4095){

        target_speed_right = 4095;

    }

}

void PID(int setpoint, int val){

    //creates variables that don't reset after each iteration of function
    static float old_error_left = 0;
    static float old_error_right = 0;

    //constants for each PID component, left and right
    float Pk_l =2, Ik_l = 0.05, Dk_l = 1;
    float Pk_r= 2, Ik_r = 0.05, Dk_r = 1;

    //how fast the PID updates, also the difference in time used in calculations
    int PID_resolution = 30;

    //components of PID
    float left_P, left_D;
    float right_P, right_D;
    static float left_I, right_I;

    //PID total values
    float PID_left, PID_right;

    left_P = Pk_l * (left_setpoint - val);
    left_I += Ik_l * ((left_setpoint - val) * PID_resolution);
    left_D = Dk_l * (((left_setpoint - val) - (old_error_left)) / PID_resolution);
    PID_left = left_P + left_I + left_D;

    right_P = Pk_r * (right_setpoint - val);
    right_I = Ik_r * ((right_setpoint - val) * PID_resolution);
    right_D = Dk_r * (((right_setpoint - val) - (old_error_right)) / PID_resolution);
    PID_right = right_P + right_I + right_D;

    old_error_left = left_P;
    old_error_right = right_P;

    output_left = PID_left;
    output_right = PID_right;

    vTaskDelay(PID_resolution / portTICK_PERIOD_MS);
}
