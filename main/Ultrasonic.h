#ifndef __ULTRASONIC_H
#define __ULTRASONIC_H

#include <NewPing.h>

class Ultrasonic {
    public:
        Ultrasonic(uint8_t triggerPin, uint8_t echoPin, unsigned int maxDist);

        // Struct for a single ultrasonic sensor
        unsigned int filteredDist;
        unsigned int curDist;
        unsigned int distBuffer1 = 0;
        unsigned int distBuffer2 = 0;
        unsigned long pingTimer;
        NewPing us;
};

#endif /* __ULTRASONIC_H */
