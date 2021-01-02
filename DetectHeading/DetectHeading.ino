#include "Arduino.h"
#include "ICM_20948.h"
#include "Quaternion.h"

#define SERIAL_PORT Serial  
#define WIRE_PORT Wire      // Wire port for I2C communication
#define WIRE_CLOCK 20e3     // I2C clock speed in Hz
#define AD0_VAL 1           // value of the last bit of the I2C address.
                            // ON the SparkFun 9DoF IMU breakout the default is
                            // 1, and when the ADR jumper is closed the value
                            // becomes 0

#define SAMPLING_PERIOD 0.02

// Globals
ICM_20948_I2C imu;  
unsigned long elapsed_time = 0;
quaternion::Quaternion orien;


void initOrientation()
{
    // Set initial pose (Use QuEst/Triad with acc & mag later)
    // For now the intial pose is always the same as the body frame of 
    // the sensor
    double yaw = 0.0;
    double pitch = 0.0;
    double roll = 0.0;

    SERIAL_PORT.print("Calibrating initial orientation...");
    orien = quaternion::euler2quat(yaw, pitch, roll);
    SERIAL_PORT.print("Done!\n");
}

void updateOrientation()
{
    //
    //  Q' = Q * exp(T/2*w)
    //
    
    // rotation axis
    double axis[3] = {deg2rad(imu.gyrX()), 
                      deg2rad(imu.gyrY()), 
                      deg2rad(imu.gyrZ())};

    // determine angle of rotation
    double norm = l2Norm(3, axis);
    double theta = sqrt(norm) * SAMPLING_PERIOD;

    // update orientation
    quaternion::Quaternion dq = quaternion::fromAxisAngle(theta, axis);
    orien = quaternion::multiply(orien, dq);
}

void printOrientation()
{
   double eulerAngles[3];
   quaternion::toEulerAngles(orien, eulerAngles);
   quaternion::printEuler2Serial(&SERIAL_PORT, eulerAngles);
}

void setup() 
{  

    // Initialize ports
    SERIAL_PORT.begin(115200);
    while(!SERIAL_PORT){};
    WIRE_PORT.begin();
    WIRE_PORT.setClock(WIRE_CLOCK);
    
    // Initialize Sensor
    bool initialised = false;
    while (!initialised)
    {
        SERIAL_PORT.print( F("Initialization of the sensor returned: ") );
        SERIAL_PORT.println( imu.statusString() );
        imu.begin(WIRE_PORT, AD0_VAL);
        if (imu.status != ICM_20948_Stat_Ok)
        {
            SERIAL_PORT.println( "Trying again..." );
            delay(500);
        }else{
            initialised = true;
        }
    }

    initOrientation();
}

void loop() 
{

    if (imu.dataReady())
    {
        while ((micros() - elapsed_time) < SAMPLING_PERIOD * 1e6) {}
        imu.getAGMT();
        elapsed_time = micros();
        updateOrientation();
        printOrientation();
    }

}
