#include <cstdint>
#include "mbed.h"
#include "Channel.h"
#ifndef FUNCTIONS_H_INCLUDED
#define FUNCTIONS_H_INCLUDED
  void readAccelData(std::int16_t *destination);  // Function prototype, its declaration
  void readGyroData(std::int16_t * destination);
  void readMagData(int16_t *destination);
#endif

// ERRORI 
//-----------------------------
int     offset_acc[3];
int     offset_gyr[3];
int     hard_mag[3];
float   soft_mag[3];

//-----------------------------

int     max[3]      = {-32768,-32768,-32768};
int     min[3]      = {32767,32767,32767};



//-------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------------------------------FUNZIONI---------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------

bool acc_gyr_calibration(int sample) {
    int bound = 400;

    int16_t acc_data[3] = {0,0,0};
    int16_t gyr_data[3] = {0,0,0};
    long    test_acc[3] = {0,0,0};
    long    test_gyr[3] = {0,0,0};
    int     max[3]      = {-32768,-32768,-32768};
    int     min[3]      = {32767,32767,32767};
    int     delta[3]    = {0,0,0};

    for (int i=0; i<sample; i++) {
        readAccelData(acc_data);
        readGyroData(gyr_data);
        
        test_acc[0] = test_acc[0] + acc_data[0];
        test_acc[1] = test_acc[1] + acc_data[1];
        test_acc[2] = test_acc[2] + acc_data[2];

        test_gyr[0] = test_gyr[0] + gyr_data[0];
        test_gyr[1] = test_gyr[1] + gyr_data[1];
        test_gyr[2] = test_gyr[2] + gyr_data[2];

        if (acc_data[0] > max[0]) {
            max[0] = acc_data[0];
        } else if (acc_data[0] < min[0]) {
            min[0]=acc_data[0]; }
        if (acc_data[1] > max[1]) {
            max[1]=acc_data[1];
        } else if (acc_data[1] < min[1]) {
            min[1]=acc_data[1]; }
        if (acc_data[2] > max[2]) {
            max[2]=acc_data[2];
        } else if (acc_data[2] < min[2]) {
            min[2]=acc_data[2]; }
        wait_ms(20);
    }
    // CALCOLIAMO LA DIFFERENZA: se troppo alta rilanceremo la calibrazione
    delta[0] = max[0] - min[0];
    delta[1] = max[1] - min[1];
    delta[2] = max[2] - min[2];

    if(delta[0]>bound || delta[1]>bound || delta[2]>bound) {  //MOVEMENT DETECTED
        return true;
    } else {    //CALIBRATION
        offset_acc[0] =  test_acc[0]/sample;
        offset_acc[1] =  test_acc[1]/sample;
        offset_acc[2] = (test_acc[2]/sample) - 16384;
        offset_gyr[0] =  test_gyr[0]/sample;
        offset_gyr[1] =  test_gyr[1]/sample;
        offset_gyr[2] =  test_gyr[2]/sample;
        return false;
    }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------

bool mag_calibration() {

    // HARD IRON CALIBRATION
    /* Faremo ruotare il sistema e registreremo massimi e minimi di ciascun asse
    finita la misurazione calcoleremo la differenza e sottrarremo il delta(hard iron) a future misurazioni*/
    int     counter     = 0;
    bool    changes     = true;
    bool    acceptable  = false;
    
    int16_t new_values[3];

    while(acceptable == false) {
        wait_ms(50);
        readMagData(new_values);
        changes = false;

        // AGGIORNA I VALORI MAX MIN
        if (new_values[0] > max[0]) {
            max[0] = new_values[0];
            changes = true;
        } else if (new_values[0]< min[0]) {
            min[0] = new_values[0];
            changes = true;  }
        if (new_values[1] > max[1]) {
            max[1] = new_values[1];
            changes = true;
        } else if (new_values[1] < min[1]) {
            min[1] = new_values[1];
            changes = true;  }
        if (new_values[2] > max[2]) {
            max[2] = new_values[2];
            changes = true;
        } else if (new_values[2] < min[2]) {
            min[2] = new_values[2];
            changes = true;  }

        // VERIFICA SE CI SONO VARIAZIONI
        if (not changes) {
            counter++;
        } else {
            counter=0;
        }
        // SE NESSUNA VARIAZIONE PER MILLE CAMPIONI FINISCI IL CAMPIONAMENTO
        if (counter==200) {
            acceptable=true;
        }
    }
    //FACCIAMO LA DIFFERENZA
    hard_mag[0] = (max[0] + min[0])/2;
    hard_mag[1] = (max[1] + min[1])/2;
    hard_mag[2] = (max[2] + min[2])/2;

    soft_mag[0] = (max[0]-hard_mag[0])/100.0000;
    soft_mag[1] = (max[1]-hard_mag[1])/100.0000;
    soft_mag[2] = (max[2]-hard_mag[2])/100.0000;

}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------

void readMagData_calibr(int16_t * destination ) {
    int16_t values[3];
    readMagData(values);
    bool changes= false;
    if (values[0] > max[0]) {
        max[0] = values[0];
        changes = true;
    } else if (values[0]< min[0]) {
        min[0] = values[0];
        changes = true;  }
    if (values[1] > max[1]) {
        max[1] = values[1];
        changes = true;
    } else if (values[1] < min[1]) {
        min[1] = values[1];
        changes = true;  }
    if (values[2] > max[2]) {
        max[2] = values[2];
        changes = true;
    } else if (values[2] < min[2]) {
        min[2] = values[2];
        changes = true;  }

    if (changes) {
        hard_mag[0] = (max[0] + min[0])/2;
        hard_mag[1] = (max[1] + min[1])/2;
        hard_mag[2] = (max[2] + min[2])/2;

        soft_mag[0] = (max[0]-hard_mag[0])/100.0000;
        soft_mag[1] = (max[1]-hard_mag[1])/100.0000;
        soft_mag[2] = (max[2]-hard_mag[2])/100.0000;
    }
    destination[0] =  values[0];
    destination[1] =  values[1];
    destination[2] =  values[2];
}


//-------------------------------------------------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------


void radio_range_calibration(Channel ch1,Channel ch2,Channel ch3,Channel ch4) {
    
}