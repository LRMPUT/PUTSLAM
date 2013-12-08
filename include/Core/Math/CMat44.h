#ifndef CMAT44_H
#define CMAT44_H
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <fstream>
using namespace std;

class CMat44
{
    public:

	/// rotation matrix
	union {
		struct {
			float m11, m12, m13;
			float m21, m22, m23;
			float m31, m32, m33;
		};
		float rot[3][3];
		float rotation[9];
	};

	/// position vector
	union {
        struct {
            float p1, p2, p3;
        };
        float pos[3];
        float p[3];
	};

	/// alternative representation of rotation
	union {
		float orientation[3];
		struct {
			float alpha, beta, gamma;
		};
		float Euler_angles[3];
	};

	CMat44();
	CMat44(float *tab);
    virtual ~CMat44();
    /// multiplication
    CMat44 operator* (CMat44 param);
	/// addition
    CMat44 operator+ (CMat44 param);
	/// subtraction
    CMat44 operator- (CMat44 param);
    /// set (row, col) element == m[row-1][col-1] = element;
    inline void setElement(float value, unsigned int row, unsigned int col){
		if ((col<4)&&(row<4))
            rotation[(row-1)*3+(col-1)] = value;
        else if ((col==4)&&(row<4))
            p[row-1] = value;
    }
    /// set initial values
    inline void setEye(void){
        setZero();
        rotation[0] = 1; rotation[4] = 1; rotation[8] = 1;
    }
	/// set initial values
    inline void setIdentity(void){
        setEye();
    }
    /// set initial values
    inline void setZero(void){
        memset(rotation, 0, sizeof(rotation));
        memset(pos, 0, sizeof(pos));
    }
    /// get element returns m[row-1][col-1] or p[row-1][col-1]
    inline float getElement(unsigned int row, unsigned int col){
        if ((col<4)&&(row<4))
            return rotation[(row-1)*3+(col-1)];
        else if ((col==4)&&(row<4))
            return p[row-1];
		else if ((col==4)&&(row==4))
            return 1;
		else
			return 0;
    }
    /// get rotation matrix as a float table
    inline void getRot(float* _rot){
        memcpy(_rot,rotation,sizeof(rotation));
    }
    /// set rotation matrix as a float table
    inline void setRot(float* _rot){
        memcpy(rotation,_rot,sizeof(rotation));
    }
    /// set position as a float table
    inline void setPos(float * _pos){
        memcpy(p,_pos, sizeof(p));
    }
    /// set position
    inline void setPos(float _x, float _y, float _z){
        pos[0] = _x; pos[1] =_y; pos[2] =_z;
    }
    /// set matrix
    inline void setMatrix(float* _rot, float* _pos){
        memcpy(rotation,_rot,sizeof(rotation));
        memcpy(pos,_pos,sizeof(pos));
    }
    /// get matrix
    inline void getMatrix(float* _rot, float* _pos){
        memcpy(_rot,rotation,sizeof(rotation));
        memcpy(_pos,p,sizeof(pos));
    }
	/// get matrix
    inline void getMatrix(float* matrix){
        matrix[0]=rotation[0]; matrix[1]=rotation[1]; matrix[2]=rotation[2];
		matrix[4]=rotation[3]; matrix[5]=rotation[4]; matrix[6]=rotation[5];
		matrix[8]=rotation[6]; matrix[9]=rotation[7]; matrix[10]=rotation[8];
		matrix[3]=pos[0]; matrix[7]=pos[1]; matrix[11]=pos[2];
        matrix[12]=0; matrix[13]=0; matrix[14]=0; matrix[15]=1;
    }
    /// copy matrix
    inline void copyMatrix(CMat44 * src){
        memcpy(this->rotation,src->rotation,sizeof(this->rotation));
        memcpy(this->pos,src->pos,sizeof(this->pos));
    }
    /// inverse of matrix
    CMat44 inv(void);
    /// inverse of matrix
    void inv(CMat44 * result);
    /// inverse of matrix
    void invThis(void);
    /// creates translation and rotation matrix
    void createTRMatrix(float alpha, float beta, float gamma, float trans_x, float trans_y, float trans_z);
    /// show matrix
    void showMatrix(void);
    ///export to file
    void exportMat2file(FILE * file);
	///export to file
	void exportMat2file(ofstream * file);
    ///load from text file
    void loadFromTextFile(FILE * file);

    protected:
    private:
};

#endif // CMAT44_H
