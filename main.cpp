
#include "mbed.h"
#include "math.h"
#include <cstdint>
#include <cstdio>
#include "MPU9150.h"
//#include "Channel.h"
#include "Calibration.h"
#include "Lights.h"
#include "FIFO_register.h"


#define PI 3.14159265

#define YAW      0
#define PITCH    1
#define ROLL     2
#define THROTTLE 3

#define X           0     // X axis
#define Y           1     // Y axis
#define Z           2     // Z axis
//CALIBRATION
int sample=200;

//raw data
int16_t raw_acc[3];
int16_t raw_gyr[3];
int16_t raw_mag[3];

//elaboreated data
float   acc_angle[3];       //angle estimation from acc
float   gyr_angle[3];       //angle estimation from acc
float   alpha = 0.99;       //first complementary filter parameter
float   beta  = 0.5;        //second complementary filter parameter
int     mag_str[3];         //intermediate variable of magnetic field
double  mag[3];             //magnetometer componets on earth system
double  pitch,roll,yaw;     //final extimeted pitch roll and yaw
int     counter = 0;        //counter
        
//MEDIA MOBILE
FifoReg fiforeg(20);
double averageYaw;

//ESC declaration and variables
int power[4] = {1000, 1000, 1000, 1000};
PwmOut ESC1(PTB0);
PwmOut ESC2(PTB1);
PwmOut ESC3(PTB2);
PwmOut ESC4(PTB3);

//RF declaration and variables
Channel channel1(PTD3,1);
Channel channel2(PTD2,2);
Channel channel3(PTD0,3);
Channel channel4(PTD5,4);
int default_offset[4] = {118, 53, 228, -12};
float default_factor[4] = {1.244344, 1.375000, 1.726845, 1.196953};

//Switch declaration
DigitalIn SW1(PTD4, PullUp);
DigitalIn SW2(PTA12,PullUp);
DigitalIn SW3(PTA4,PullUp);
DigitalIn SW4(PTA5,PullUp);
DigitalIn SW5(PTC8,PullUp);

//Cycle Timer
Timer CycleTimer;
long CycleBegin, CycleEnd;
int CycleCounter=0;
long CycleTime;

//Serial port
bool serialCom;
Serial pc(USBTX,USBRX,9600);

//SWITCHES
//
// SW 1     serial output
// SW 2     Accelerometer-Gyroscope calibration
// SW 3     Magnetometer hand calibration
// SW 4     radio calibration
// SW 5     pid calibration //START
// SW 6     Sound On Off


//_______________________________________________________________________________________________________________________________________
//
//|||||||||||||||||||||||||||||||||||||||||||||||||   MAIN   |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
//_______________________________________________________________________________________________________________________________________

int main() 
{   
    //blink(1, 5, 500);
    //SERIAL DECLARATION if (serialCom) {  }
    signal_start();//SOUND AND LIGHT EFFECT
    wait(1);

    if (not SW1){
        printf("SWITCH_1 ON:\nSerial comunication ON\n\nSTARTING SETUP \n\n");
        serialCom = true;
        signal_ok();wait(1);//SOUND AND LIGHT EFFECT
    } else { serialCom = false; }


    //INIZIALIZZAZIONE 
    
    float test_mag[3];
    initMPU9150();
    initAK8975A(test_mag);
    if (serialCom) { printf("MPU-9150 and AK8975A initialized \n\n");}
    signal_ok();wait(1);//SOUND AND LIGHT EFFECT


    //CALIBRAZIONE ACC E GYRO
    if (not SW2){
        if (serialCom) { printf("SWITCH_2 ON\n\nSTARTING ACC/GYRO CALIBRATION IN 3 SECONDS \nPLEASE DON'T MOVE! \n\n"); }
        signal_calibration(1);//SOUND AND LIGHT EFFECT
        bool movement=1;
        while (movement) {
            movement=acc_gyr_calibration(sample);
            if (movement) {
                signal_error(); //SOUND AND LIGHT EFFECT
                if (serialCom){printf("MOVEMENT DETECTED: Restarting \n\n");}
                signal_calibration(1);//SOUND AND LIGHT EFFECT
            }
        }
        signal_stop_calibration(1); wait(1);//SOUND AND LIGHT EFFECT
        if (serialCom) {printf("OFFSET CALCULATED \nACC: %d %d %d   GYRO: %d %d %d \n\n\n", offset_acc[0],offset_acc[1],offset_acc[2],offset_gyr[0],offset_gyr[1],offset_gyr[2]); }
    } else {
        if (serialCom) {printf("SWITCH_2 OFF\n\n");}
    }

    //CALIBRAZIONE MAG
    if (not SW3){
        signal_calibration(2);//SOUND AND LIGHT EFFECT
        if (serialCom) {printf("SWITCH 3 ON\n\nMAGNETOMETER CALIBRATION: Rotate DRONE in all direction \n\n\n"); }

        mag_calibration();
        if (serialCom) {printf("Hard Iron OFFSET CALCULATED\nHARD IRON: %5d %5d %5d\nsoft IRON: %8f %8f %8f\n\n\n",
                                hard_mag[0],hard_mag[1],hard_mag[2],soft_mag[0],soft_mag[1],soft_mag[2]);}
        signal_stop_calibration(2);//SOUND AND LIGHT EFFECT
    } else {
        if (serialCom) {printf("SWITCH 3 OFF\n\n");}
    }
    
    
    //CALIBRAZIONE RADIO
    if (not SW4){
        signal_calibration(3);//SOUND AND LIGHT EFFECT
        if (serialCom) {printf("SWITCH 4 ON\n\nRADIO INPUT CALIBRATION: Rotate JOYSTICKS in all direction \n\n\n"); }

        for( int j=0; j<300; j++) {
            wait_ms(20);
            channel1.find_interval();
            channel2.find_interval();
            channel3.find_interval();
            channel4.find_interval();
        }
        channel1.calculate_calibration();wait_ms(20);
        channel2.calculate_calibration();wait_ms(20);
        channel3.calculate_calibration();wait_ms(20);
        channel4.calculate_calibration();wait_ms(20);

        if (serialCom) {printf("Radio Input calibrated\n");wait(0.1);
            channel1.get_parameters();wait(0.1);
            channel2.get_parameters();wait(0.1); 
            channel3.get_parameters();wait(0.1);
            channel4.get_parameters();wait(0.1);
        }
        signal_stop_calibration(3);//SOUND AND LIGHT EFFECT
    } else {
        if (serialCom) {printf("SWITCH 4 OFF\n");}
        channel1.channel_default_value(default_offset[0], default_factor[0]);
        channel2.channel_default_value(default_offset[1], default_factor[1]);
        channel3.channel_default_value(default_offset[2], default_factor[2]);
        channel4.channel_default_value(default_offset[3], default_factor[3]);
    }
 
    signal_ready();wait(1);//SOUND AND LIGHT EFFECT
    shutup();wait(1);//SHUTUP

    //MOTOR 
    if (serialCom) {printf("STARTING ESC \n\n");}
    ESC1.period_ms(20);
    ESC2.period_ms(20);
    ESC3.period_ms(20);
    ESC4.period_ms(20);

    if (serialCom) {printf("SHUT SW5 TO START \n\n");}
    while(not SW5) {
        wait_ms(500);
    }
    wait(1);

    ESC1.pulsewidth_us(power[0]);
    ESC2.pulsewidth_us(power[1]);
    ESC3.pulsewidth_us(power[2]);
    ESC4.pulsewidth_us(power[3]);
    wait(2);

    power[0] = 1100;
    power[1] = 1100;
    power[2] = 1100;
    power[3] = 1100;
    if (serialCom) {printf("START \n\n");}

    


//_______________________________________________________________________________________________________________________________________
//
//|||||||||||||||||||||||||||||||||||||||||||||||||  LOOP  |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
//_______________________________________________________________________________________________________________________________________

    while(1) {
        //Cycle time counter
        CycleTimer.start();
        CycleBegin = CycleTimer.read_us();
        
        // read raw data from sensors
        readAccelData(raw_acc);
        readGyroData(raw_gyr);
        //readMagData(raw_mag);
        readMagData_calibr(raw_mag); //same of readMagData(raw_mag) with in-flight autocalibration added

        // Accelerometer correction
        raw_acc[0] -= offset_acc[0];
        raw_acc[1] -= offset_acc[1];
        raw_acc[2] -= offset_acc[2];

        // Gyroscope offset correction
        raw_gyr[0] -= offset_gyr[0];
        raw_gyr[1] -= offset_gyr[1];
        raw_gyr[2] -= offset_gyr[2];

        // Magnetometer hard and soft iron correction
        raw_mag[0] = (raw_mag[0] - hard_mag[0]) / soft_mag[0];      // per correggere l'errore si è sviluppato il modello
        raw_mag[1] = (raw_mag[1] - hard_mag[1]) / soft_mag[1];      // X_rilevato = X_offset + alpha * x_corretto
        raw_mag[2] = (raw_mag[2] - hard_mag[2]) / soft_mag[1];      // per risolvere soft iron si normalizza a 100
    
        // correct axsis orientation of magnetometer
        mag_str[0] =  raw_mag[1];
        mag_str[1] =  -raw_mag[0];
        mag_str[2] =  raw_mag[2];

        // get angle aproximation by accelerometer vector
        acc_angle[Y]    = -atan2(  raw_acc[X],  sqrtf( raw_acc[Y] * raw_acc[Y]  +  raw_acc[Z] * raw_acc[Z] )  ) ;
        acc_angle[X]    = -atan2(  raw_acc[Y],  sqrtf( raw_acc[X] * raw_acc[X]  +  raw_acc[Z] * raw_acc[Z] )  ) ;

        // get angle aproximation by angular speed time integration (in radiant (0.0174533))
        gyr_angle[Y]   += (( raw_gyr[Y] * 1.000000 ) * CycleTime / 1000000 / 65.5) * 0.0174533;
        gyr_angle[X]   += ((-raw_gyr[X] * 1.000000 ) * CycleTime / 1000000 / 65.5) * 0.0174533;

        // compensate gyro angle with accelerometer angle in a complementary filter (accelerometer -> LF ; gyroscope -> HF)
        gyr_angle[Y]    = gyr_angle[Y] * alpha + acc_angle[Y] * (1-alpha);
        gyr_angle[X]    = gyr_angle[X] * alpha + acc_angle[X] * (1-alpha);

        // compensate yawing motion in angle estimation
        gyr_angle[Y]   += gyr_angle[X] * sin( ( ( raw_gyr[Z] * 1.000000 ) * CycleTime / 1000000 / 65.5) * 0.0174533 );
        gyr_angle[X]   -= gyr_angle[Y] * sin( ( ( raw_gyr[Z] * 1.000000 ) * CycleTime / 1000000 / 65.5) * 0.0174533 );

        // get pitch and roll (low pass complementary filter)
        pitch  = pitch * beta + gyr_angle[Y] * (1-beta);
        roll   = roll  * beta + gyr_angle[X] * (1-beta);

        // get yaw estimation by magnetometer 
        mag[0] = mag_str[0] * cos(pitch) + mag_str[1] * sin(roll) * sin(pitch) - mag_str[2] * cos(roll) * sin(pitch);
        mag[1] = mag_str[1] * cos(roll)  + mag_str[2] * sin(roll);
        yaw    = atan2(mag[1], mag[0]) ;


        printf("%11f %11f %11f  -  %11f %11f  - %11f %11f \n",pitch*180/PI,roll*180/PI,yaw*180/PI,gyr_angle[Y]*180/PI,gyr_angle[X]*180/PI,acc_angle[Y]*180/PI,acc_angle[X]*180/PI);


        //applicazione della media mobile
        averageYaw = fiforeg.FifoReg_shift_and_m_av(yaw* 180 /PI);

        //Insert complementary filter 
        








        //THROTTLE POWER CALCULATION
        channel1.calibratre();
        channel2.calibratre();
        channel3.calibratre();
        channel4.calibratre();

        power[0] = 1000 + channel3.read();
        power[1] = 1000 + channel3.read();
        power[2] = 1000 + channel3.read();
        power[3] = 1000 + channel3.read();

        //channel1.print();channel2.print();channel3.print();channel4.print();
        //printf("\n");
        
        //ESC OUTPUT
        ESC1.pulsewidth_us(power[0]);
        ESC2.pulsewidth_us(power[1]);
        ESC3.pulsewidth_us(power[2]);
        ESC4.pulsewidth_us(power[3]);
        //printf("%6d %6d %6d   \n",raw_mag[0],raw_mag[1],raw_mag[2]);
        /*
        //printf("%6d %6d %6d  -  %6d %6d %6d  -  %6d %6d %6d\n",raw_acc[0],raw_acc[1],raw_acc[2],raw_gyr[0],raw_gyr[1],raw_gyr[2],raw_mag[0],raw_mag[1],raw_mag[2]);
        //printf("%6d %6d %6d   \n",raw_mag[0],raw_mag[1],raw_mag[2]);
        //printf("%11f, %11f, %11f  \n",pitch, roll, yaw);
        if (not SW4 && not SW3) {
            printf("%11f, %11f, %11f       %11f, \n",pitch* 180 /PI, roll* 180 /PI, yaw* 180 /PI, averageYaw);
        } else if (not SW4 && SW3) {
            //printf("%13f, %13f        %9f, %9f, %9f\n",mag[0], mag[1], pitch, roll, yaw);

        } else if (SW4 && not SW3) {
            //printf("%6d %6d %6d    -    %5f\n",raw_mag[0],raw_mag[1],raw_mag[2],intensity);
            //fiforeg.FifoReg_print();
        } else {
            
        }
        
        */
        //printf("HARD IRON: %5d %5d %5d    soft IRON: %6f %6f %6f \n", hard_mag[0],hard_mag[1],hard_mag[2], soft_mag[0],soft_mag[1],soft_mag[2]);
        //printf("%11f, %11f, %11f       %11f, \n",pitch* 180 /PI, roll* 180 /PI, yaw* 180 /PI, averageYaw);
        CycleEnd = CycleTimer.read_us();
        CycleTime = CycleEnd - CycleBegin;
        //printf("Cycle: %d \n", CycleEnd - CycleBegin);
        CycleTimer.reset();
        CycleCounter++;
        if (CycleCounter==1000)
        {
            CycleCounter=0;
            on_off(2);
        }
    }
}


