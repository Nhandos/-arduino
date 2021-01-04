// Runs the intialisation routine

#ifndef STATE_INIT_H
#define STATE_INIT_H
#include "Arduino.h"
#include "StateVector.h"

#define SERIAL_PORT Serial  
#define WIRE_PORT Wire      // Wire port for I2C communication
#define WIRE_CLOCK 20e3     // I2C clock speed in Hz
#define AD0_VAL 1           // value of the last bit of the I2C address.
                            // ON the SparkFun 9DoF IMU breakout the default is
                            // 1, and when the ADR jumper is closed the value
                            // becomes 0


#define GYR_DLPF_ENABLE false 
#define ACC_DLPF_ENABLE false 
#define MAX_ICM209248_CONNECTION_ATTEMPTS 5

// Globals are defined here
ICM_20948_I2C imu;  
quaternion::Quaternion orien;

typedef enum 
{
    INIT_OK = 0x00,
    INIT_HARDWARE_ERR,
    INIT_CFG_ERR
} Init_Stat_e;

const char* initStatusString(Init_Stat_e val)
{
    switch(val)
    {
        case INIT_OK:
            return "OK";
            break;
        case INIT_HARDWARE_ERR:
            return "Hardware error";
            break;
        case INIT_CFG_ERR:
            return "Configuration error";
            break;

        default:
            return "Invalid status";
    }
}

Init_Stat_e initOrientation()
{
    Init_Stat_e retval = INIT_OK;

    // Set initial pose (Use QuEst/Triad with acc & mag later)
    // For now the intial pose is always the same as the body frame of 
    // the sensor
    double yaw = 0.0;
    double pitch = 0.0;
    double roll = 0.0;

    SERIAL_PORT.print("Calibrating initial orientation...");
    orien = quaternion::euler2quat(yaw, pitch, roll);
    // print orientation
    double eulerAngles[3];
    quaternion::toEulerAngles(orien, eulerAngles);
    quaternion::printEuler2Serial(&SERIAL_PORT, eulerAngles);
    SERIAL_PORT.print("Done!\n");

    return retval;
}

Init_Stat_e initPorts()
{
    Init_Stat_e retval = INIT_OK;

    // Initialize ports
    SERIAL_PORT.begin(115200);
    while(!SERIAL_PORT){};
    WIRE_PORT.begin();
    WIRE_PORT.setClock(WIRE_CLOCK);

    return retval;
}

Init_Stat_e initICM20948()
{
    // Initialize Sensor
    Init_Stat_e init_ret = INIT_OK;
    int n_tries = 0;
    bool initialised = false;
    while (!initialised)
    {
        SERIAL_PORT.print( F("Initialization of the sensor returned: ") );
        SERIAL_PORT.println( imu.statusString() );
        imu.begin(WIRE_PORT, AD0_VAL);
        if (imu.status != ICM_20948_Stat_Ok)
        {
            if (n_tries == MAX_ICM209248_CONNECTION_ATTEMPTS)
            {
                SERIAL_PORT.println( "Max attempt to connect reached");
                init_ret = INIT_HARDWARE_ERR;
                return init_ret;
            }

            SERIAL_PORT.println( "Trying again..." );
            delay(500);
            n_tries ++;
        }else{
            initialised = true;
        }
    }

    // Configure Sensor
    // SW reset to make sure the device starts in a known state
    imu.swReset( );
    if( imu.status != ICM_20948_Stat_Ok){
        SERIAL_PORT.print(F("Software Reset returned: "));
        SERIAL_PORT.println(imu.statusString());
        init_ret = INIT_CFG_ERR;
    }
    delay(250);

    // Now wake the sensor up
    imu.sleep( false );
    imu.lowPower( false );
    imu.startupMagnetometer(); //magnetometer need to be restarted after sleep
    delay(250);

    // Set Gyro and Accelerometer to a particular sample mode 
    // options: ICM_20948_Sample_Mode_Continuous
    // ICM_20948_Sample_Mode_Cycled
    imu.setSampleMode( (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), 
                        ICM_20948_Sample_Mode_Continuous ); 
    if( imu.status != ICM_20948_Stat_Ok){
        SERIAL_PORT.print(F("setSampleMode returned: "));
        SERIAL_PORT.println(imu.statusString());
        init_ret = INIT_CFG_ERR;
    }

    // Set full scale ranges for both acc and gyr
    ICM_20948_fss_t myFSS;  // This uses a "Full Scale Settings" structure that
                            // can contain values for all configurable sensors

    myFSS.a = gpm2;       // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
                          // gpm2
                          // gpm4
                          // gpm8
                          // gpm16
                          
    myFSS.g = dps250;     // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
                          // dps250
                          // dps500
                          // dps1000
                          // dps2000
                          
    imu.setFullScale( (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), 
                      myFSS );  
    if( imu.status != ICM_20948_Stat_Ok){
        SERIAL_PORT.print(F("setFullScale returned: "));
        SERIAL_PORT.println(imu.statusString());
        init_ret = INIT_CFG_ERR;
    }


    // Set up Digital Low-Pass Filter configuration
    ICM_20948_dlpcfg_t myDLPcfg;            
    myDLPcfg.a = acc_d473bw_n499bw;     // (ICM_20948_ACCEL_CONFIG_DLPCFG_e)
                                        // acc_d246bw_n265b 
                                        // acc_d111bw4_n136bw
                                        // acc_d50bw4_n68bw8
                                        // acc_d23bw9_n34bw4
                                        // acc_d11bw5_n17bw
                                        // acc_d5bw7_n8bw3 
                                        // acc_d473bw_n499bw

    myDLPcfg.g = gyr_d361bw4_n376bw5;   // (ICM_20948_GYRO_CONFIG_1_DLPCFG_e)
                                        // gyr_d196bw6_n229bw8
                                        // gyr_d151bw8_n187bw6
                                        // gyr_d119bw5_n154bw3
                                        // gyr_d51bw2_n73bw3
                                        // gyr_d23bw9_n35bw9
                                        // gyr_d11bw6_n17bw8
                                        // gyr_d5bw7_n8bw9
                                        // gyr_d361bw4_n376bw5
                                          
    imu.setDLPFcfg( (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), 
                    myDLPcfg );
    if( imu.status != ICM_20948_Stat_Ok){
        SERIAL_PORT.print(F("setDLPcfg returned: "));
        SERIAL_PORT.println(imu.statusString());
        init_ret = INIT_CFG_ERR;
    }

    // Choose whether or not to use DLPF
    ICM_20948_Status_e retval;

    retval = imu.enableDLPF( ICM_20948_Internal_Acc, ACC_DLPF_ENABLE);
    if (retval != ICM_20948_Stat_Ok)
    {
        SERIAL_PORT.print(F("Enable DLPF for Accelerometer returned: ")); 
        SERIAL_PORT.println(imu.statusString(retval));
        init_ret = INIT_CFG_ERR;
    }

    retval = imu.enableDLPF( ICM_20948_Internal_Gyr, GYR_DLPF_ENABLE);
    if (retval != ICM_20948_Stat_Ok)
    {
        SERIAL_PORT.print(F("Enable DLPF for Gyroscope returned: ")); 
        SERIAL_PORT.println(imu.statusString(retval));
        init_ret = INIT_CFG_ERR;
    }

    SERIAL_PORT.println();
    SERIAL_PORT.println(F("Configuration complete!")); 

    return init_ret;
}

Init_Stat_e run_init()
{
    Init_Stat_e retval;

    retval = initPorts();
    if (retval != INIT_OK)
    {
        SERIAL_PORT.print(F("Initialising ports returned: "));
        SERIAL_PORT.println(initStatusString(retval));
        return retval;
    }

    retval = initICM20948();
    if (retval != INIT_OK)
    {
        SERIAL_PORT.print(F("Initialising ICM20948 returned: "));
        SERIAL_PORT.println(initStatusString(retval));

        if (retval == INIT_HARDWARE_ERR)
        {
            SERIAL_PORT.println("aborting due to hardware error");
            exit(0);
        }
    }

    retval = initOrientation() ;
    if (retval != INIT_OK)
    {
        SERIAL_PORT.print(F("Initialising ICM20948 returned: "));
        SERIAL_PORT.println(initStatusString(retval));
    }

    // State transition
    SERIAL_PORT.println(F("Transitioning to Filtering State"));
    g_state = FILTERING_STATE;
}

#endif
