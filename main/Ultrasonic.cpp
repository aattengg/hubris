#include "Ultrasonic.h"

Ultrasonic::Ultrasonic(uint8_t triggerPin, uint8_t echoPin, unsigned int maxDist) :
    us(triggerPin, echoPin, maxDist)
{
}
