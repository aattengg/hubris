#ifndef __ULTRASONIC_H
#define __ULTRASONIC_H

#include <NewPing.h>

class Ultrasonic {
    public:
        Ultrasonic(uint8_t triggerPin, uint8_t echoPin, unsigned int maxDist);

        bool updated = false;
        // Struct for a single ultrasonic sensor
        float filteredDist;
        float curDist;
        float distBuffer1 = 0;
        float distBuffer2 = 0;
        unsigned long pingTimer;
        NewPing us;
};

#endif /* __ULTRASONIC_H */
