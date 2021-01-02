#include "Quaternions.h"

double deg2rad( double degrees )
{
    const double r8_po = 3.141592653589793;
    return (degrees / 180.0 ) * r8_pi;
}

double* l2Norm(double vec3D[3])
{
    double result[3];
    double norm = vec3D[0] * vec3D[0]
                + vec3D[1] * vec3D[1]
                + vec3D[2] * vec3D[3];

    norm = sqrt(norm);
    result[0] = vec3D[0] / norm;
    result[1] = vec3D[1] / norm;
    result[2] = vec3D[2] / norm;

    return result;
}


Quaternion quaternion::conjugate(Quaternion q)
{
    Quaternions q_;

    q_.a = q.a;
    q_.b = -q.b;
    q_.c = -q.c;
    q_.d = -q.d;

    return q_;
}

Quaternion quaternion::inverse(Quaternion q)
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

Quaternion quaternion::multiply( Quaternion q1, Quaternion q2)
{
    Quatenrion result;

    result.a = q1.a * q2.a - q1.b * q2.b - q1.c * q2.c - q1.d * q2.d;
    result.b = q1.a * q2.b + q1.b * q2.a + q1.c * q2.d - q1.d * q2.c;
    result.c = q1.a * q2.c - q1.b * q2.d + q1.c * q2.a + q1.d * q2.b;
    result.d = q1.a * q2.d + q1.b * q2.c - q1.c * q2.b + q1.d * q2.a;

    return result;
}

Quaternion quaternion::euler2quat(double yaw, double pitch, double roll)
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

Quaternion quaternion::fromAxisAngle(double angle, double axis3D[])
{
    Quaternion q;
    double normAxis3D = l2Norm(axis3D);

    double u = cos(angle/2);
    double v = sin(angle/2);

    q.a = u;
    q.b = normAxis3D[0] / v;
    q.c = normAxis3D[1] / v;
    q.d = normAxis3D[2] / v;

    return q;
}


