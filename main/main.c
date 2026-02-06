

////////DEVICE 1/////////////////////////
//Modify file
//git add .
//git commit -m "Describe your changes"
//git push

////////Device 2/////////////////////////
//git pull


#include <stdio.h>
#include "driver/adc.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"

#define button 14
#define servo 13
#define IN1 16
#define IN2 17
#define IN3 18
#define IN4 19
#define EN1 26
#define EN2 27

int xybPins[] = {13, 25, 14};
int xVal, yVal, bVal;
int duty_steering;
int duty_throttle;

void input_init();
void servo_init();
void getVal();
void set_servo_DC();
void set_throttle_DC();
void printVal_background(void *pvParameters);
void fwd();
void rev();
void brake();

void app_main(){

    //Initialize inputs
    input_init();

    //Initialize PWM for the servo
    servo_init();

    //Start the background printing process
    xTaskCreate(printVal_background, "Live Values", 2048, NULL, 5, NULL);

    while(1){

        getVal();
        set_servo_DC();
        set_throttle_DC();

        if(yVal > 2950){

            fwd();

        }

        if(yVal < 2850){

            rev();

        }

        if(yVal > 2850 && yVal < 2950){

            brake();

        }

        vTaskDelay(10 / portTICK_PERIOD_MS);

    }

}

void input_init(){

    printf("Joystick program starting...\n");

    //Reset the button pin to the default state
    gpio_reset_pin(button);
    gpio_reset_pin(servo);

    // Set ADC width to 12 bits
    adc1_config_width(ADC_WIDTH_BIT_12);
    
    // Configure ADC channels pin 36
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_12);

    // Configure ADC channels pin 39
    adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_12);

    // Configure pin 14 as input for button
    gpio_set_direction(button, GPIO_MODE_INPUT);

    //Set pull-up resistor on the button pin (0v -> resistor -> 3.3v; when pressed)
    gpio_set_pull_mode(button, GPIO_PULLUP_ONLY);

    //Configure pin 16 as output for IN1
    gpio_set_direction(IN1, GPIO_MODE_OUTPUT);

    //Configure pin 17 as output for IN2
    gpio_set_direction(IN2, GPIO_MODE_OUTPUT);

    //Configure pin 18 as output for IN3
    gpio_set_direction(IN3, GPIO_MODE_OUTPUT);

    //Configure pin 19 as output for IN4
    gpio_set_direction(IN4, GPIO_MODE_OUTPUT);

    //Configure pin 26 as output for EN1
    gpio_set_direction(EN1, GPIO_MODE_OUTPUT);

    //Configure pin 27 as output for IN4
    gpio_set_direction(EN2, GPIO_MODE_OUTPUT);

}

void servo_init(){

    // Setup the timer for the PWM
    ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_12_BIT,  // 12-bit = 0-4095
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 50,                         // 50 Hz for servos
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_config);

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

    // Setup the PWM channel for EN1 (left motor)
    ledc_channel_config_t channel_config2 = {
        .gpio_num = EN1,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&channel_config2);
    
    // Setup the PWM channel for EN2 (right motor)
    ledc_channel_config_t channel_config3 = {
        .gpio_num = EN2,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&channel_config3);

}

void getVal(){

    // Read ADC values for pin 36 (x value) and pin 39 (y value)
    xVal = adc1_get_raw(ADC1_CHANNEL_0);
    yVal = adc1_get_raw(ADC1_CHANNEL_3);

    // Read digital value for pin 14 (button)
    bVal = gpio_get_level(button);

}

void set_servo_DC(){

//Duty cycles from trial and error
//Mid point: 282
//Full clockwise: 80
//Full counter clockwise: 493

    //Rested X values for ADC tested from serial monitor (1820 - 1840 for 3.3v)
    if(xVal > 2800 &&  xVal < 2900){

        // Tested value from serial monitor
        duty_steering = 282; //Mid point: ((80 + 493) / 2)    

    }

    else{

        // Map the joystick x value reading to 12 bit pwm range (Trial and error extremes)

        //for 3.3V
        duty_steering = 80 + ((xVal * (493 - 80)) / 4095);

        //for 5.5V
        //duty_steering = xVal;

    }

    // Stores the new duty cycle variable in memory
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty_steering);
    // Sets the steering duty cycle to "duty_steering"
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

}

void set_throttle_DC(){

    //Rested Y values for ADC tested from serial monitor
    if(yVal >= 2850 && yVal <= 2950){

        // Stops the motor
        duty_throttle = 0;

    }

    if(yVal < 2850){

        //Maps 0-2950 to PWM duty cycle scale (0-4095)
        //duty_throttle = ((abs(2900 - yVal)) / 2900) * 4905; rewritten to avoid truncation
        //2900 is the more accurate rested y value
        duty_throttle = ((abs(2900 - yVal)) * 4905) / 2900;

    }

    if(yVal > 2950){

        //Maps 2900 - 4095 to PWM duty cycle scale (0-4095) (0 - 4095)
        //duty_throttle = ((yVal - 2900) / (4095 - 2900)) * 4095; rewritten to avoid truncation
        //2900 is the more accurate rested y value
        duty_throttle = ((yVal - 2900) * 4095) / (4095 - 2900);

    }

    // Stores the new duty cycle variable in memory
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, duty_throttle);
    // Sets the throttle PWM duty cycle to "duty_throttle"
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);

    // Stores the new duty cycle variable in memory
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, duty_throttle);
    // Sets the throttle PWM duty cycle to "duty_throttle"
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);

}

    void printVal_background(void *pvParameters){

    while(1){

        // print values with updates instead of new lines (\r vs \n)
        printf("\rX: %4d, Y: %4d, Button: %d, Steering Duty Cycle: %4d, Throttle Duty Cycle: %4d", 
            xVal, yVal, bVal, duty_steering, duty_throttle);
        //vTaskDelay(1000 / portTICK_PERIOD_MS);

        // Clear print for new line
        fflush(stdout);

        // Change delay from 50 <-> 200 ms for better response, but no printing of values
        vTaskDelay(150 / portTICK_PERIOD_MS);

    }

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

    gpio_set_level(IN1, 1);
    gpio_set_level(IN2, 1);

}