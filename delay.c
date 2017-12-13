#include "delay.h"

void delayMicroSecond (unsigned int time)
{
    while (time--)
    {
        __delay_cycles(1);
    }
}

void delayMiliSecond  (unsigned int time)
{
    while (time--)
    {
        __delay_cycles(MILI_SECOND);
    }
}
