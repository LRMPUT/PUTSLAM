#ifndef CQUATERNION_H
#define CQUATERNION_H
#include <stdio.h>

class CQuaternion
{
    public:
        CQuaternion();
        virtual ~CQuaternion();

		///quaternion vector
        union{
            float quat[4];
            struct {
                float q1, q2, q3, q4;
            };
			struct {
                float sw, sx, sy, sz;
            };
            float q[4];
            struct {
                float w, x, y, z;
            };
        };
		
		/// set default values
		inline void Reset(){
			x = 0.0;    y = 0.0;    z = 0.0;    w = 1.0;
		}
		///copy quaternion
		inline void copyQuat(CQuaternion q){
			x = q.x;    y = q.y;    z = q.z;    w = q.w;
		}
		/// set value
		inline void set(float _sx, float _sy, float _sz, float _sw) {
			x = _sx; y = _sy; z = _sz; w = _sw;
		}
        /// copy sign to value
        float copySign(float value, float sign);
        /// show quaternions
        void showQuat(void);
		/// axis and angle to quaternion
		void axisAngleToQuat(const float *axis, float theta);
		/// Euler angles to quaternion
        void EulerToQuat(float pitch, float yaw, float roll);
		/// normalise quaternion
        void normaliseQuat();
		/// compute magnitude
        float magnitudeQuat();
		/// multiply quaternions
		void multQuat(CQuaternion q);
        ///export quaternion to file
        void quat2file(FILE * file);
        ///export to file
        void exportQuat2file(FILE * file);

    protected:
    private:
};

#endif // CQUATERNION_H
