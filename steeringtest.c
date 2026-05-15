void steering(){

    //FORWARD THROTTLE MAPPING
    if(throttle_val > 2700){

        float scratch = (float)(throttle_val - 2700) / (4095 - 2700);
        duty_throttle = scratch * (410-307) + 307;
        
    }

    //REVERSE THROTTLE MAPPING
    else if (throttle_val < 2100){

        float scratch = (float)(2100 - throttle_val) / 2100;
        duty_throttle = 307 - scratch * (307-205);

    }

    //NEUTRAL THROTTLE MAPPING

    else{

        duty_throttle = 307;

    }

    //ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty_throttle);
    //ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1); 

}