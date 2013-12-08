#ifndef _Vector_H
#define _Vector_H

#include <math.h>

class CVector
{
    public:
           CVector(float sx = 0, float sy = 0, float sz = 0);
          ~CVector();

        //vector
		union {
			struct {
				float x, y, z;
			};
			float v[3];
		};

		/// returns magnitude
        float getMagnitude();
		/// set default values
        inline void reset() {
			x = 0;    y = 0;    z = 0;
		}
		/// set values
        void set(float sx, float sy, float sz) {x = sx, y = sy, z = sz;}
		/// cross product
        void crossVector(CVector vect);
		/// dot product
        float dotProduct(CVector vect);
        /// equal within an error ‘e’
        inline const bool nearlyEquals( const CVector& v, const float e ) const {
            return ((fabs(x-v.x)<e) && (fabs(y-v.y)<e) && (fabs(z-v.z)<e));
        }
        /// cross product
        inline const CVector cross( const CVector& v ) const {
            return CVector( y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x );
        }
        /// scalar dot product
        inline const float dot( const CVector& v ) const {
            return x*v.x + y*v.y + z*v.z;
        }
        /// length
        inline const float length() const {
            return (float)sqrt( (double)this->dot(*this) );
        }
        /// unit vector
        inline const CVector unit() const {
            return (*this) / length();
        }
        /// make this a unit vector
        inline void normalize() {
            (*this) /= length();
        }
        /// index a component
        float& operator [] ( const long i ) {
            return *((&x) + i);
        }
        /// compare
        const bool operator == ( const CVector& v ) const {
            return (v.x==x && v.y==y && v.z==z);
        }
		/// not equal
        const bool operator != ( const CVector& v ) const {
            return !(v == *this);
        }
        /// negate
        const CVector operator - () const {
            return CVector( -x, -y, -z );
        }
        /// assign
        const CVector& operator = ( const CVector& v ) {
            x = v.x; y = v.y; z = v.z;
            return *this;
        }
        /// increment
        const CVector& operator += ( const CVector& v ) {
            x+=v.x; y+=v.y; z+=v.z;
            return *this;
        }
        /// decrement
        const CVector& operator -= ( const CVector& v ) {
            x-=v.x; y-=v.y; z-=v.z;
            return *this;
        }
        /// self-multiply
        const CVector& operator *= ( const float& s ) {
            x*=s; y*=s; z*=s;
            return *this;
        }
        /// self-divide
        const CVector& operator /= ( const float& s ) {
            const float r = 1 / s;
            x *= r; y *= r; z *= r;
            return *this;
        }
        /// add
        const CVector operator + ( const CVector& v ) const {
            return CVector(x + v.x, y + v.y, z + v.z);
        }
        /// subtract
        const CVector operator - ( const CVector& v ) const {
            return CVector(x - v.x, y - v.y, z - v.z);
        }
        /// post-multiply by a float
        const CVector operator * ( const float& s ) const {
            return CVector( x*s, y*s, z*s );
        }
        /// pre-multiply by a float
        friend inline const CVector operator * ( const float& s, const CVector& v ) {
            return v * s;
        }
        /// divide
        const CVector operator / (float s) const {
            s = 1/s;
            return CVector( s*x, s*y, s*z );
        }
};

#endif

