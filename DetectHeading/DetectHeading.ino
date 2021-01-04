#include "Arduino.h"
#include "ICM_20948.h"
#include "Quaternion.h"
#include "Initialisation.h"
#include "StateVector.h"


#define SAMPLING_PERIOD 0.02  // 50Hz sampling rate

// TODO: Move filter to it's own file
void runFilter()
{
    static unsigned long sample_t = millis();
    unsigned long delta_t = millis() - sample_t;
    
    if (imu.dataReady() && delta_t > SAMPLING_PERIOD * 1e3)
    {
        imu.getAGMT();
        sample_t = millis();

        // get rotation axis
        double axis[3] = {deg2rad(imu.gyrX()), 
                          deg2rad(imu.gyrY()), 
                          deg2rad(imu.gyrZ())};

        // determine angle of rotation
        double norm = l2Norm(3, axis);
        double theta = sqrt(norm) * delta_t * 1e-3;

        // update orientation
        quaternion::Quaternion dq = quaternion::fromAxisAngle(theta, axis);
        orien = quaternion::multiply(orien, dq);

    }

    // print orientation
    double eulerAngles[3];
    quaternion::toEulerAngles(orien, eulerAngles);
    quaternion::printEuler2Serial(&SERIAL_PORT, eulerAngles);
        
}


void setup() 
{  
    g_state = INITIALISATION_STATE;
}

void runCycle()
{

    switch(g_state)
    {
        case INITIALISATION_STATE:
            run_init();
            break;

        case FILTERING_STATE:
            runFilter(); 
            break;

        case STANDBY_STATE:
            break;

        default:
            SERIAL_PORT.println("Fatal error - Invalid state");
            exit(0);
    }

}

void loop()
{
    runCycle();
}
