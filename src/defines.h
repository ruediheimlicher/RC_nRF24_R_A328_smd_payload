//
//  expo.h
//  DOG_LCD
//
//  Created by Ruedi Heimlicher on 22.11.2013.
//
//

#include <inttypes.h>


#ifndef DEFINES_h
#define DEFINES_h

#define TEST    1
#define R_SMD 0
#define  R_DIL 1
#define BOARD 1

#define LOOPLED_DDR     DDRD
#define LOOPLED_PORT    PORTD
#define LOOPLED         PD6

#define BATT_DDR        DDRC
#define BATT_PORT       PORTC
#define BATT_PIN        PC3



#define BLINKRATE 0x04FF

#define FIRSTTIMEDELAY  0x0FF
#define RADIOSTARTED    1
#define RADIORUNNING    2

#define MITTE 170

// SMD
#define S0  PD0     // PD0 // YAW
#define S1  PD1     // PD1 // PITCH
#define S2  PD2     // PD2 // ROLL
#define S3  PD3     // PD3 // THROTTLE
#define IO0 PD4     // PD4 // AUX

#define OSZIA_DDR       DDRD
#define OSZIA_PORT      PORTD
#define OSZIA_PIN       PD4

#define OSZIAHI         OSZIA_PORT |= (1<<OSZIA_PIN)
#define OSZIALO         OSZIA_PORT &= ~(1<<OSZIA_PIN)


#define CE_PIN 10   // PB2
#define CSN_PIN 9  // PB1

#endif


