#ifndef QUATERNIONS_H
#define QUATERNIONS_H

#include <Arduino.h>
#include <math.h>

double deg2rad( double degrees );
double l2Norm(int dims, double vec3D[3]);

// Express a quaternion of the form
//      q = a + bi + cj + dk
namespace quaternion{
    typedef struct quaternion_t
    {
        double a, b, c, d;
    } Quaternion;

    Quaternion conjugate(Quaternion q);
    Quaternion inverse(Quaternion q);
    Quaternion multiply( Quaternion q1, Quaternion q2);
    Quaternion euler2quat(double yaw, double pitch, double roll);
    void toEulerAngles(Quaternion q, double *angles);
    Quaternion fromAxisAngle(double angle, double axis3D[]);
    void printQuat2Serial(Quaternion q);
    void printEuler2Serial(double* eulerAnlges);
}

#endif
