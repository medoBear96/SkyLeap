#include "mbed.h"
#include <cstdint>


class FifoReg{
    private:
        int length;
        float* regist;

    public:
        FifoReg(int len) {
            this->length=len;
            regist = new float[length];
        }

        void FifoReg_shift(float new_val) {
            for (int i=length-1; i>1; i--) {
                regist[i] = regist[i-1];
            }
             regist[0] = new_val;
        }

        double FifoReg_shift_and_m_av(float new_val) {
            volatile double average=0.000000;
            for (int i=length-1; i>=1; i--) {
                regist[i] = regist[i-1];
                average += regist[i];
            }
            regist[0] = new_val;
            average += regist[0];
            average /= length;


            return average;
        }

        void FifoReg_print() {
            for (int i=length-1; i>=0; i--) {
                printf("%6f ",regist[i]);

            }
            printf("\n ");
        }
        
};