#include "../include/Core/Math/CQuaternion.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

CQuaternion::CQuaternion()
{
	quat[0]=1; quat[1]=0; quat[2]=0; quat[3]=0;
    //ctor
}

CQuaternion::~CQuaternion()
{
    //dtor
}

/// copy sign to value
float CQuaternion::copySign(float value, float sign){
    if (sign>0) return fabs(value);
    else if (sign<0) return -fabs(value);
    else return 0;
}

/// axis and angle to quaternion
void CQuaternion::axisAngleToQuat(const float *axis, float theta)
{
	float halfTheta = theta * 0.5;
	float cosHalfTheta = cos(halfTheta);
	float sinHalfTheta = sin(halfTheta);
	x = axis[0] * sinHalfTheta;
	y = axis[1] * sinHalfTheta;
	z = axis[2] * sinHalfTheta;
	w = cosHalfTheta;
}

/// Euler angles to quaternion
void CQuaternion::EulerToQuat(float roll, float pitch, float yaw)
{
	float cr, cp, cy, sr, sp, sy, cpcy, spsy;  // calculate trig identities
	cr = cos(roll/2);
	cp = cos(pitch/2);
	cy = cos(yaw/2);
    sr = sin(roll/2);
    sp = sin(pitch/2);
    sy = sin(yaw/2);
    cpcy = cp * cy;
    spsy = sp * sy;
    w = cr * cpcy + sr * spsy;
    x = sr * cpcy - cr * spsy;
    y = cr * sp * cy + sr * cp * sy;
    z = cr * cp * sy - sr * sp * cy;
}

/// normalise quaternion
void CQuaternion::normaliseQuat()
{
	float Mag;
	Mag = magnitudeQuat();
	w = w/Mag;
	x = x/Mag;
	y = y/Mag;
	z = z/Mag;
}

/// compute magnitude
float CQuaternion::magnitudeQuat()
{
    return( sqrt(w*w+x*x+y*y+z*z));
}

/// multiply quaternions
void CQuaternion::multQuat(CQuaternion q)
{
	CQuaternion q3;
	float vectorq1[3] = {x, y, z};
	float vectorq2[3] = {q.x, q.y, q.z};

    float tempvec1[3], tempvec2[3], tempvec3[3];
	memcpy(tempvec1, vectorq1, sizeof(tempvec1));
    q3.w = (w*q.w) - (tempvec1[0]*vectorq2[0]+tempvec1[1]*vectorq2[1]+tempvec1[2]*vectorq2[2]);
	tempvec1[0] = tempvec1[1] * vectorq2[2] - tempvec1[2] * vectorq2[1];
	tempvec1[1] = tempvec1[2] * vectorq2[0] - tempvec1[0] * vectorq2[2];
	tempvec1[2] = tempvec1[0] * vectorq2[1] - tempvec1[1] * vectorq2[0];
    tempvec2[0] = w * q.x;
    tempvec2[1] = w * q.y;
    tempvec2[2] = w * q.z;
    tempvec3[0] = q.w * x;
    tempvec3[1] = q.w * y;
    tempvec3[2] = q.w * z;
    q3.x = tempvec1[0] + tempvec2[0] + tempvec3[0];
    q3.y = tempvec1[1] + tempvec2[1] + tempvec3[1];
    q3.z = tempvec1[2] + tempvec2[2] + tempvec3[2];
    copyQuat(q3);
}

/// show quaternions
void CQuaternion::showQuat(void){
    printf("w=%f, x=%f, y=%f, z=%f\n",w, x, y, z);
}

///export quaternion to file
void CQuaternion::quat2file(FILE * file){
    fprintf(file, "quat=[%f %f %f %f];\n", quat[0], quat[1], quat[2], quat[3]);
}
