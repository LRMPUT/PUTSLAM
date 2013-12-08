#include "../include/Core/Math/vector.h"
#include <cmath>

CVector::CVector(float sx, float sy, float sz)
:
    x(sx),
    y(sy),
    z(sz)
{
}

CVector::~CVector() {
}

/// dot product
float CVector::dotProduct(CVector vect){
      return vect.x * x + vect.y * y + vect.z * z;
}

/// cross product
void CVector::crossVector(CVector vect) {
      x = vect.y * this->z - vect.z * this->y;
      y = vect.z * this->x - vect.x * this->z;
      z = vect.x * this->y - vect.y * this->x;
}

/// returns magnitude
float CVector::getMagnitude() {
    float magnitude = (float)sqrt(x * x + y * y + z * z);
    if (magnitude != 0.0f)
        return magnitude;
    else
        return 1.0;
}
