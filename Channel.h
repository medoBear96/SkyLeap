#include "mbed.h"
#include <cstdint>

/*
this class take care of radio reciever input
The signal comes as a PWM of 50 Hz frequency and a duty cycle of 5to10% 
an interrupt signal the rise and fall of the pwm wave to a timer. 
The interval is corrected and remapped in the range [0,1000]us.
*/
class Channel{
private:
    InterruptIn interrupt;
    Timer timer;
    volatile int time;
    int number;     //name id 

    
    int max, min;   //time boundaries
    int offset; 
    int new_max;
    float factor;
    int calibrated;

public:
    Channel(PinName pin, int num) : interrupt(pin) { //constructor
        this->number=num;
        interrupt.rise(this, &Channel::start);  
        interrupt.fall(this, &Channel::stop);
        offset=1000; min=1000; max=0;
        factor=1;
    }

    void start() { timer.start(); }
    
    void stop() {
        timer.stop();
        time  = timer.read_us()-1000;
        timer.reset();
    }
    
    int read() { return calibrated; }

    void find_interval(){
            if (time > max){ max = time; } else
            if (time < min){ min = time; }
    }
    void calculate_calibration(){
        offset = min+50;
        new_max=max-offset;
        factor = 1100.00000/new_max;
    }
    int calibratre() { 
        calibrated = (time-offset)*factor;
        if (calibrated>1000) {calibrated=1000;} else 
        if (calibrated<0) {calibrated=0;} else 
        if (calibrated<515 && calibrated>485){ calibrated=500; }
        return calibrated;
    }

    void print() { printf("CH%d: %5d    ",number, calibrated); }

    int calibrate_read() { 
        calibrated = (time-offset)*factor;
        return calibrated;
    }

    void get_parameters() {
        printf("Offset:%8d  new_max:%6d  factor:%10f \n", offset,new_max,factor);
    }


};

