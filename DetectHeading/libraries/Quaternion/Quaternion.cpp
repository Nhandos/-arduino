#include "Quaternion.h"

static void printFormattedFloat(HardwareSerial *port, float val, uint8_t leading, uint8_t decimals);

double deg2rad( double degrees )
{
    const double r8_pi = 3.141592653589793;
    return (degrees / 180.0 ) * r8_pi;
}

double rad2deg( double radians )
{
    const double r8_pi = 3.141592653589793;
    return (radians / r8_pi ) * 180.0;
}

double l2Norm(int dims, double* vec3D)
{
    double norm = 0.0;
    for (int i = 0; i < dims; i++)
    {
        norm += vec3D[i] * vec3D[i];
    }

    return sqrt(norm);
}


namespace quaternion {
    Quaternion conjugate(Quaternion q)
    {
        Quaternion q_;

        q_.a = q.a;
        q_.b = -q.b;
        q_.c = -q.c;
        q_.d = -q.d;

        return q_;
    }

    Quaternion inverse(Quaternion q)
    {
       double norm;
       Quaternion result;

       norm = q.a * q.a
            + q.b * q.b
            + q.c * q.c
            + q.d * q.d;

       result.a =  q.a / norm;
       result.b = -q.b / norm;
       result.c = -q.c / norm;
       result.d = -q.d / norm;

        return result;
    }

    Quaternion multiply( Quaternion q1, Quaternion q2)
    {
        Quaternion result;

        result.a = q1.a * q2.a - q1.b * q2.b - q1.c * q2.c - q1.d * q2.d;
        result.b = q1.a * q2.b + q1.b * q2.a + q1.c * q2.d - q1.d * q2.c;
        result.c = q1.a * q2.c - q1.b * q2.d + q1.c * q2.a + q1.d * q2.b;
        result.d = q1.a * q2.d + q1.b * q2.c - q1.c * q2.b + q1.d * q2.a;

        return result;
    }

    Quaternion euler2quat(double yaw, double pitch, double roll)
    { 
        double cy = cos(yaw * 0.5);
        double sy = sin(yaw * 0.5);
        double cp = cos(pitch * 0.5);
        double sp = sin(pitch * 0.5);
        double cr = cos(roll * 0.5);
        double sr = sin(roll * 0.5);

        Quaternion q;
        q.a = cr * cp * cy + sr * sp * sy;
        q.b = sr * cp * cy - cr * sp * sy;
        q.c = cr * sp * cy + sr * cp * sy;
        q.d = cr * cp * sy - sr * sp * cy;

        return q;
    }

    Quaternion fromAxisAngle(double angle, double axis3D[])
    {
        Quaternion q;
        double norm = l2Norm(3, axis3D);

        if (norm == 0.0)
        {
            Serial.print("quaternion::fromAxisAngle - Fatal error!\n");
            Serial.print("  The axis vector is null.\n");
            exit(1);
        }

        double u = cos(angle/2);
        double v = sin(angle/2);

        q.a = u;
        q.b = axis3D[0] * v / norm;
        q.c = axis3D[1] * v / norm;
        q.d = axis3D[2] * v / norm;

        return q;
    }

    void toEulerAngles(Quaternion q, double *angles)
    {

        // roll (x-axis rotation)
        double sinr_cosp = 2 * (q.a * q.b + q.c * q.d);
        double cosr_cosp = 1 - 2 * (q.b * q.b + q.c * q.c);
        angles[0] = atan2(sinr_cosp, cosr_cosp);

        // pitch (y-axis rotation)
        double sinp = 2 * (q.a * q.c - q.d * q.b);
        if (fabs(sinp) >= 1)
        {
            angles[1] = sinp / fabs(sinp) * M_PI / 2;
        }
        else{
            angles[1] = asin(sinp);
        }

        // yaw (z-axis rotation)
        double siny_cosp = 2 * (q.a * q.d + q.b * q.c);
        double cosy_cosp = 1 - 2 * (q.c * q.c + q.d * q.d);
        angles[2] = atan2(siny_cosp, cosy_cosp);
    }

    void printQuat2Serial(HardwareSerial *port, Quaternion q)
    { 
        port->print("[ ");
        printFormattedFloat(port, q.a, 3, 2);
        port->print(", ");
        printFormattedFloat(port, q.b, 3, 2);
        port->print(", ");
        printFormattedFloat(port, q.c, 3, 2);
        port->print(", ");
        printFormattedFloat(port, q.d, 3, 2);
        port->println(" ]");
    }

    void printEuler2Serial(HardwareSerial* port, double* eulerAngles)
    {
        port->print("[ Roll: ");
        printFormattedFloat(port, rad2deg(eulerAngles[0]), 3, 2);
        port->print(", Pitch: ");
        printFormattedFloat(port, rad2deg(eulerAngles[1]), 3, 2);
        port->print(", Yaw: ");
        printFormattedFloat(port, rad2deg(eulerAngles[2]), 3, 2);
        port->println(" ]");

    }


} // namespace quaternion

static void printFormattedFloat(HardwareSerial *port, float val, uint8_t leading, uint8_t decimals){
  float aval = abs(val);
  if(val < 0){
    port->print("-");
  }else{
    port->print(" ");
  }
  for( uint8_t indi = 0; indi < leading; indi++ ){
    uint32_t tenpow = 0;
    if( indi < (leading-1) ){
      tenpow = 1;
    }
    for(uint8_t c = 0; c < (leading-1-indi); c++){
      tenpow *= 10;
    }
    if( aval < tenpow){
      port->print("0");
    }else{
      break;
    }
  }
  if(val < 0){
    port->print(-val, decimals);
  }else{
    port->print(val, decimals);
  }
}



