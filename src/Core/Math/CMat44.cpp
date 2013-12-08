#include "../include/Core/Math/CMat44.h"
#include <math.h>

CMat44::CMat44()
{
    memset(pos, 0, sizeof(pos));
    setEye();
}

CMat44::CMat44(float *tab){
	rot[0][0] = tab[0]; rot[0][1] = tab[1]; rot[0][2] = tab[2];
	rot[1][0] = tab[4]; rot[1][1] = tab[5]; rot[1][2] = tab[6];
	rot[2][0] = tab[8]; rot[2][1] = tab[9]; rot[2][2] = tab[10];
	pos[0] = tab[3]; pos[1] = tab[7]; pos[2] = tab[11];
}

CMat44::~CMat44()
{
    //dtor
}

CMat44 CMat44::operator* (CMat44 param) {
  CMat44 temp;
  temp.rotation[0] = rotation[0]*param.rotation[0] + rotation[1]*param.rotation[3] + rotation[2]*param.rotation[6];
  temp.rotation[1] = rotation[0]*param.rotation[1] + rotation[1]*param.rotation[4] + rotation[2]*param.rotation[7];
  temp.rotation[2] = rotation[0]*param.rotation[2] + rotation[1]*param.rotation[5] + rotation[2]*param.rotation[8];

  temp.rotation[3] = rotation[3]*param.rotation[0] + rotation[4]*param.rotation[3] + rotation[5]*param.rotation[6];
  temp.rotation[4] = rotation[3]*param.rotation[1] + rotation[4]*param.rotation[4] + rotation[5]*param.rotation[7];
  temp.rotation[5] = rotation[3]*param.rotation[2] + rotation[4]*param.rotation[5] + rotation[5]*param.rotation[8];

  temp.rotation[6] = rotation[6]*param.rotation[0] + rotation[7]*param.rotation[3] + rotation[8]*param.rotation[6];
  temp.rotation[7] = rotation[6]*param.rotation[1] + rotation[7]*param.rotation[4] + rotation[8]*param.rotation[7];
  temp.rotation[8] = rotation[6]*param.rotation[2] + rotation[7]*param.rotation[5] + rotation[8]*param.rotation[8];

  temp.pos[0] = rotation[0]*param.pos[0] + rotation[1]*param.pos[1] + rotation[2]*param.pos[2] + pos[0];
  temp.pos[1] = rotation[3]*param.pos[0] + rotation[4]*param.pos[1] + rotation[5]*param.pos[2] + pos[1];
  temp.pos[2] = rotation[6]*param.pos[0] + rotation[7]*param.pos[1] + rotation[8]*param.pos[2] + pos[2];

  return (temp);
}

///addition
CMat44 CMat44::operator+ (CMat44 param) {
  CMat44 temp;
  for (size_t i=0; i<9 ;i++)
	temp.rotation[i] = rotation[i] + param.rotation[i];

  for (size_t i=0; i<3 ;i++)
	temp.pos[i] = pos[i] + param.pos[i];

  return (temp);
}

///subtraction
CMat44 CMat44::operator- (CMat44 param) {
  CMat44 temp;
  for (size_t i=0; i<9 ;i++)
	temp.rotation[i] = rotation[i] - param.rotation[i];

  for (size_t i=0; i<3 ;i++)
	temp.pos[i] = pos[i] - param.pos[i];

  return (temp);
}

/// creates translation and rotation matrix according to euler angles and transition vector
void CMat44::createTRMatrix(float alpha, float beta, float gamma, float trans_x, float trans_y, float trans_z){
	setEye();

	CMat44 rotX,rotY,rotZ;

	rotX.setEye();
	rotY.setEye();
	rotZ.setEye();


	rotX.setElement(cos(alpha),2,2);
	rotX.setElement(-sin(alpha),2,3);
	rotX.setElement(sin(alpha),3,2);
	rotX.setElement(cos(alpha),3,3);

	rotY.setElement(cos(beta),1,1);
	rotY.setElement(sin(beta),1,3);
	rotY.setElement(-sin(beta),3,1);
	rotY.setElement(cos(beta),3,3);

	rotZ.setElement(cos(gamma),1,1);
	rotZ.setElement(-sin(gamma),1,2);
	rotZ.setElement(sin(gamma),2,1);
	rotZ.setElement(cos(gamma),2,2);

	*this=rotX*rotY*rotZ;
	setElement(trans_x,1,4);
	setElement(trans_y,2,4);
	setElement(trans_z,3,4);

	orientation[0]=alpha; orientation[1]=beta; orientation[2]=gamma;
}

/// inverse of matrix
CMat44 CMat44::inv(void){
    CMat44 temp;
	//R'
	temp.setElement(getElement(1,1),1,1);
	temp.setElement(getElement(1,2),2,1);
	temp.setElement(getElement(1,3),3,1);

	temp.setElement(getElement(2,1),1,2);
	temp.setElement(getElement(2,2),2,2);
	temp.setElement(getElement(2,3),3,2);

	temp.setElement(getElement(3,1),1,3);
	temp.setElement(getElement(3,2),2,3);
	temp.setElement(getElement(3,3),3,3);

	//R'*p
	temp.setElement(-(temp.getElement(1,1)*getElement(1,4)+temp.getElement(1,2)*getElement(2,4)+temp.getElement(1,3)*getElement(3,4)),1,4);
	temp.setElement(-(temp.getElement(2,1)*getElement(1,4)+temp.getElement(2,2)*getElement(2,4)+temp.getElement(2,3)*getElement(3,4)),2,4);
	temp.setElement(-(temp.getElement(3,1)*getElement(1,4)+temp.getElement(3,2)*getElement(2,4)+temp.getElement(3,3)*getElement(3,4)),3,4);

	return temp;
}

/// inverse of matrix
void CMat44::inv(CMat44 * result){
    *result = inv();
}

/// inverse of this matrix
void CMat44::invThis(void){
	(*this) = inv();
}

void CMat44::showMatrix(void){
    printf("%f %f %f | %f\n",m11, m12, m13, pos[0]);
    printf("%f %f %f | %f\n",m21, m22, m23, pos[1]);
    printf("%f %f %f | %f\n",m31, m32, m33, pos[2]);
}

///export to file
void CMat44::exportMat2file(FILE * file){
    fprintf(file, "mat=[%f %f %f %f; %f %f %f %f; %f %f %f %f; %f %f %f %f];\n", rotation[0], rotation[1], rotation[2], pos[0], rotation[3], rotation[4], rotation[5], pos[1], rotation[6], rotation[7], rotation[8], pos[2], 0.0, 0.0, 0.0, 1.0);
}

///export to file
void CMat44::exportMat2file(ofstream * file){
    *file << rotation[0] << " " << rotation[1] << " " << rotation[2] << " " << pos[0] << " " << rotation[3] << " " << rotation[4] << " " << rotation[5] << " " << pos[1] << " " << rotation[6] << " " << rotation[7] << " " << rotation[8] << " " << pos[2] << " " << 0.0 << " " << 0.0 << " " << 0.0 << " " << 1.0 << " ";
}

///load from text file
void CMat44::loadFromTextFile(FILE * file){
    float tmp;
    int reti;
    if (fscanf(file, "mat=[")) reti = 0;
    if (fscanf(file, "%f ",&rotation[0])) reti = 0; if (fscanf(file, "%f ",&rotation[1])) reti = 0; if (fscanf(file, "%f ",&rotation[2])) reti = 0; if (fscanf(file, "%f;",&pos[0])) reti = 0;
    if (fscanf(file, "%f ",&rotation[3])) reti = 0; if (fscanf(file, "%f ",&rotation[4])) reti = 0; if (fscanf(file, "%f ",&rotation[5])) reti = 0; if (fscanf(file, "%f;",&pos[1])) reti = 0;
    if (fscanf(file, "%f ",&rotation[6])) reti = 0; if (fscanf(file, "%f ",&rotation[7])) reti = 0; if (fscanf(file, "%f ",&rotation[8])) reti = 0; if (fscanf(file, "%f;",&pos[2])) reti = 0;
    if (fscanf(file, "%f ",&tmp)) reti = 0; if (fscanf(file, "%f ",&tmp)) reti = 0; if (fscanf(file, "%f ",&tmp)) reti = 0; if (fscanf(file, "%f];\n",&tmp)) reti = 0;
}
