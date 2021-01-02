#ifndef QUATERNIONS_H
#define QUATERNIONS_H

// Express a quaternion of the form
//      q = a + bi + cj + dk
typedef struct quaternion_t
{
    double a, b, c, d;
} Quaternion;

double deg2rad( double degrees );
double* l2Norm(double vec3D[3]);
Quaternion quaternion::conjugate(Quaternion q);
Quaternion quaternion::inverse(Quaternion q);
Quaternion quaternion::multiply( Quaternion q1, Quaternion q2);
Quaternion quaternion::euler2quat(double yaw, double pitch, double roll);

#endif
