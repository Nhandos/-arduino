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


void configureIMU()
{
    // Here we are doing a SW reset to make sure the device starts in a known state
    imu.swReset( );
    if( imu.status != ICM_20948_Stat_Ok){
    SERIAL_PORT.print(F("Software Reset returned: "));
    SERIAL_PORT.println(imu.statusString());
    }
    delay(250);

    // Now wake the sensor up
    imu.sleep( false );
    imu.lowPower( false );
    imu.startupMagnetometer(); //magnetometer need to be restarted after sleep
    delay(250);

    // The next few configuration functions accept a bit-mask of sensors for which the settings should be applied.

    // Set Gyro and Accelerometer to a particular sample mode
    // options: ICM_20948_Sample_Mode_Continuous
    //          ICM_20948_Sample_Mode_Cycled
    imu.setSampleMode( (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous ); 
    if( imu.status != ICM_20948_Stat_Ok){
    SERIAL_PORT.print(F("setSampleMode returned: "));
    SERIAL_PORT.println(imu.statusString());
    }

    // Set full scale ranges for both acc and gyr
    ICM_20948_fss_t myFSS;  // This uses a "Full Scale Settings" structure that can contain values for all configurable sensors

    myFSS.a = gpm2;         // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
                          // gpm2
                          // gpm4
                          // gpm8
                          // gpm16
                          
    myFSS.g = dps250;       // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
                          // dps250
                          // dps500
                          // dps1000
                          // dps2000
                          
    imu.setFullScale( (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS );  
    if( imu.status != ICM_20948_Stat_Ok){
    SERIAL_PORT.print(F("setFullScale returned: "));
    SERIAL_PORT.println(imu.statusString());
    }


    // Set up Digital Low-Pass Filter configuration
    ICM_20948_dlpcfg_t myDLPcfg;            // Similar to FSS, this uses a configuration structure for the desired sensors
    myDLPcfg.a = acc_d473bw_n499bw;         // (ICM_20948_ACCEL_CONFIG_DLPCFG_e)
                                          // acc_d246bw_n265bw      - means 3db bandwidth is 246 hz and nyquist bandwidth is 265 hz
                                          // acc_d111bw4_n136bw
                                          // acc_d50bw4_n68bw8
                                          // acc_d23bw9_n34bw4
                                          // acc_d11bw5_n17bw
                                          // acc_d5bw7_n8bw3        - means 3 db bandwidth is 5.7 hz and nyquist bandwidth is 8.3 hz
                                          // acc_d473bw_n499bw

    myDLPcfg.g = gyr_d5bw7_n8bw9;       // (ICM_20948_GYRO_CONFIG_1_DLPCFG_e)
                                          // gyr_d196bw6_n229bw8
                                          // gyr_d151bw8_n187bw6
                                          // gyr_d119bw5_n154bw3
                                          // gyr_d51bw2_n73bw3
                                          // gyr_d23bw9_n35bw9
                                          // gyr_d11bw6_n17bw8
                                          // gyr_d5bw7_n8bw9
                                          // gyr_d361bw4_n376bw5
                                          
    imu.setDLPFcfg( (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myDLPcfg );
    if( imu.status != ICM_20948_Stat_Ok){
    SERIAL_PORT.print(F("setDLPcfg returned: "));
    SERIAL_PORT.println(imu.statusString());
    }

    // Choose whether or not to use DLPF
    // Here we're also showing another way to access the status values, and that it is OK to supply individual sensor masks to these functions
    ICM_20948_Status_e accDLPEnableStat = imu.enableDLPF( ICM_20948_Internal_Acc, false );
    ICM_20948_Status_e gyrDLPEnableStat = imu.enableDLPF( ICM_20948_Internal_Gyr, false );
    SERIAL_PORT.print(F("Enable DLPF for Accelerometer returned: ")); SERIAL_PORT.println(imu.statusString(accDLPEnableStat));
    SERIAL_PORT.print(F("Enable DLPF for Gyroscope returned: ")); SERIAL_PORT.println(imu.statusString(gyrDLPEnableStat));

    SERIAL_PORT.println();
    SERIAL_PORT.println(F("Configuration complete!")); 
}

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

    // In this advanced example we'll cover how to do a more fine-grained setup of your sensor
    SERIAL_PORT.println("Device connected!");


    configureIMU();
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
