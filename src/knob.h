#ifndef KNOB_H
#define KNOB_H

#include <Arduino.h>
#include <STM32FreeRTOS.h>

class Knob {
    volatile uint8_t prev = 0;
    volatile uint8_t current = 0;
    volatile int8_t rotation_increment = 0; // can be negative
    volatile int8_t rotation_variable = 0; // from 0 to 16

    uint8_t max = 16;
    uint8_t min = 0;
    uint8_t num = 3;
    /*
        0: 4,C2,C3
        1: 4,C0,C1
        2: 3,C2,C3
        3: 3,C0,C1
    */

   void readKnob() { // Now only supports knob 3: okay
        bool b0 = (prev >> 1) & B1;
        bool a0 = (prev >> 0) & B1;
        bool b1 = (current >> 1) & B1;
        bool a1 = (current >> 0) & B1;
        int8_t local_rotation_increment;

        if (b0 == b1 && a1 == a0) {
            // no change 00->00, 01->01, 10->10, 11->11
            rotation_increment = 0;
        } else if (b0 != b1 &&  a1 != a0) {
            // impossible transition: 00->11, 01->10, 10->01, 11->00
            //  q6a: interpret impossible transitions by assuming impossible transition is same as last legal transition: so increase or decrease by 2
            if (rotation_increment == 1 || rotation_increment == 2) {
            local_rotation_increment = 2;
            } else if (rotation_increment == -1 || rotation_increment == -2) {
            local_rotation_increment = -2;
            }
        } else if (prev == 0 && current == 1 || prev == 1 && current == 3 || prev == 2 && current == 0 || prev == 3 && current == 2) {
            // +1: 00->01, 01->11, 10->00, 11->10
            local_rotation_increment = 1;
        } else if (prev == 0 && current == 2 || prev == 1 && current == 0 || prev == 2 && current == 3 || prev == 3 && current == 1) {
            // -1: 00->10, 01->00, 10->11, 11->01
            local_rotation_increment = -1;
        }
        __atomic_store_n(&rotation_increment,local_rotation_increment,__ATOMIC_RELAXED);
    }

    void updateRotation() {
        int8_t local_rotation_variable;
        if (rotation_variable + rotation_increment > max) {
            local_rotation_variable = max;
        } else if (rotation_variable + rotation_increment < min) {
            local_rotation_variable = min;
        } else {
            local_rotation_variable = rotation_variable + rotation_increment;
        }
        __atomic_store_n(&rotation_variable,local_rotation_variable,__ATOMIC_RELAXED);
    }

    public:
        Knob(uint8_t maximum, uint8_t minimum, uint8_t knob_num) {
            max = maximum;
            min = minimum;
            num = knob_num;
        }

        void read(uint8_t input) {
            if (num == 3 || num == 1 ) {
                current = input; //For Now
            } else if (num == 2 || num == 0 ) {
                current = input >> 2;
            }
        }

        void update() {
            readKnob();
            updateRotation();
            prev = current;
        }

        int8_t get_rotation() {
            return rotation_variable;
        }

        int8_t get_rotation_increment() {
            return rotation_increment;
        }
};

#endif