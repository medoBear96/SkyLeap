#include "mbed.h"
#include <cstdint>


class FifoReg{
    private:
        int length;
        int* regist;

    public:
        FifoReg(int len) {
            this->length=len;
            regist = new int[length];
        }

        void FifoReg_shift(int new_val) {
            for (int i=length-1; i>1; i--) {
                regist[i] = regist[i-1];
            }
             regist[0] = new_val;
        }

        int FifoReg_shift_and_m_av(int new_val) {
            volatile int average=0;
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
                printf("%6d ",regist[i]);

            }
            printf("\n ");
        }
        
};