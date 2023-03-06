#include "mbed.h"
#include <cstdint>


#define MPU9150_ADDRESS     0x68<<1
#define AK8975A_ADDRESS     0x0C<<1
#define AK8975A_ST1         0x02
#define AK8975A_XOUT_L      0x03  // data
#define AK8975A_XOUT_H      0x04
#define AK8975A_YOUT_L      0x05
#define AK8975A_YOUT_H      0x06
#define AK8975A_ZOUT_L      0x07
#define AK8975A_ZOUT_H      0x08
#define AK8975A_ST2         0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8975A_CNTL        0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8975A_ASTC        0x0C  // Self test control
#define AK8975A_ASAX        0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8975A_ASAY        0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8975A_ASAZ        0x12  // Fuse ROM z-axis sensitivity adjustment value
#define ACCEL_XOUT_H        0x3B
#define ACCEL_XOUT_L        0x3C
#define ACCEL_YOUT_H        0x3D
#define ACCEL_YOUT_L        0x3E
#define ACCEL_ZOUT_H        0x3F
#define ACCEL_ZOUT_L        0x40
#define TEMP_OUT_H          0x41
#define TEMP_OUT_L          0x42
#define GYRO_XOUT_H         0x43
#define GYRO_XOUT_L         0x44
#define GYRO_YOUT_H         0x45
#define GYRO_YOUT_L         0x46
#define GYRO_ZOUT_H         0x47
#define GYRO_ZOUT_L         0x48
#define PWR_MGMT_1          0x6B // Device defaults to the SLEEP mode
#define CONFIG              0x1A
#define GYRO_CONFIG         0x1B
#define ACCEL_CONFIG        0x1C
#define INT_PIN_CFG         0x37
#define INT_ENABLE          0x38
#define DMP_INT_STATUS      0x39  // Check DMP interrupt
#define INT_STATUS          0x3A
#define SMPLRT_DIV          0x19


I2C i2c(PTE0, PTE1);

//Accelerometer range selector
enum Ascale {
    AFS_2G = 0,
    AFS_4G,
    AFS_8G,
    AFS_16G
    };

//Gyroscope range selector
enum Gscale {
    GFS_250DPS = 0,
    GFS_500DPS,
    GFS_1000DPS,
    GFS_2000DPS
    };

void writeByte(uint8_t address, uint8_t regster, uint8_t data) {
    char data_write[2];
    data_write[0] = regster;
    data_write[1] = data;
    i2c.write(address, data_write, 2, 0);
    }

char readByte(uint8_t address, uint8_t regster)  {
    char data_read[1];    
    char data_write[1];
    data_write[0] = regster;
    i2c.write(address, data_write, 1, 1); 
    i2c.read(address, data_read, 1, 0); 
    return data_read[0]; 
    }

// read multiple Bytes
void readBytes(uint8_t address, uint8_t regster, uint8_t count, uint8_t * dest) {     
    char data_read[14];
    char data_write[1];
    data_write[0] = regster;
    i2c.write(address, data_write, 1, 1);                       // no stop
    i2c.read(address, data_read, count, 0); 
    for(int i = 0; i < count; i++) {
        dest[i] = data_read[i];
        }
    }
    
void initAK8975A(float * destination) {
    uint8_t rawData[3];                                         // raw data stored here
    writeByte(AK8975A_ADDRESS, AK8975A_CNTL, 0x00);             // Power down
    wait(0.01);
    writeByte(AK8975A_ADDRESS, AK8975A_CNTL, 0x0F);             // Enter Fuse ROM access mode
    wait(0.01);
    readBytes(AK8975A_ADDRESS, AK8975A_ASAX, 3, &rawData[0]);   // Read values
    destination[0] =  (float)(rawData[0] - 128)/256.0f + 1.0f;  
    destination[1] =  (float)(rawData[1] - 128)/256.0f + 1.0f;  
    destination[2] =  (float)(rawData[2] - 128)/256.0f + 1.0f; 
    }
    
void readAccelData(int16_t * destination) {
    uint8_t rawData[6];                                                     // x/y/z accel register data stored here
    readBytes(MPU9150_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);               // Read the six raw data registers into data array
    destination[0] = (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;   // Turn the MSB and LSB into a signed 16-bit value
    destination[1] = (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
    destination[2] = (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
    }

void readGyroData(int16_t * destination) {
    uint8_t rawData[6];                                                     // x/y/z gyro register data stored here
    readBytes(MPU9150_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);                // Read the six raw data registers sequentially into data array
    destination[0] = (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;   // Turn the MSB and LSB into a signed 16-bit value
    destination[1] = (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
    destination[2] = (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
    }

void readMagData(int16_t * destination ) {
    uint8_t rawData[6];                                 // x/y/z gyro register data stored here
    int16_t cachedata[3];
    writeByte(AK8975A_ADDRESS, AK8975A_CNTL, 0x01);     // toggle enable data read from magnetometer, no continuous read mode!
    while(not(readByte(AK8975A_ADDRESS, AK8975A_ST1) & 0x01)) {
        wait_us(100);
    }
    readBytes(AK8975A_ADDRESS, AK8975A_XOUT_L, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
    cachedata[0]   = ((int16_t)rawData[1] << 8) | rawData[0] ;   // Turn the MSB and LSB into a signed 16-bit value
    cachedata[1]   = ((int16_t)rawData[3] << 8) | rawData[2] ;  
    cachedata[2]   = ((int16_t)rawData[5] << 8) | rawData[4] ; 
    destination[0] =  cachedata[0];
    destination[1] =  cachedata[1];
    destination[2] =  cachedata[2];
    }

void initMPU9150() {                                // wake up device
    writeByte(MPU9150_ADDRESS, PWR_MGMT_1, 0x00);   // Clear sleep mode bit (6), enable all sensors 
    wait(0.2);                                      // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt  

    // get stable time source
    writeByte(MPU9150_ADDRESS, PWR_MGMT_1, 0x01);  // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
    wait(0.2);
    
    // Configure Gyro and Accelerometer
    // Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively; 
    // DLPF_CFG = bits 2:0 = 010; this sets the sample rate at 1 kHz for both
    // Minimum delay time is 4.9 ms which sets the fastest rate at ~200 Hz
    writeByte(MPU9150_ADDRESS, CONFIG, 0x03);  
    
    // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
    writeByte(MPU9150_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; the same rate set in CONFIG above
    
    // Set gyroscope full scale range
    // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
    
    // Configure Interrupts and Bypass Enable
    // Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips 
    // can join the I2C bus and all can be controlled by the Arduino as master
    writeByte(MPU9150_ADDRESS, INT_PIN_CFG, 0x22);    
    writeByte(MPU9150_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
    }
    