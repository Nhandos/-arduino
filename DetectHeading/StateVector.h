#ifndef STATE_VECTOR_H
#define STATE_VECTOR_H

#include "Arduino.h"

typedef enum state_id
{

    INITIALISATION_STATE = 0x00,
    FILTERING_STATE,
    STANDBY_STATE

} State_ID_e;

// Global
State_ID_e g_state = STANDBY_STATE;

#endif


