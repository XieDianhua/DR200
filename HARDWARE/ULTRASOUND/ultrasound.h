#ifndef _ULTRASOUND_H
#define _ULTRASOUND_H

#include "sys.h"
#include "system.h"

//Trig1 PC6 
//Econ  PD15
//Trig2 PD14
//Econ  PD13

void Ultrasound_Init(u16 arr, u16 psc);
void Ultrasonic_Rang(void);


#endif


