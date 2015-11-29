#include "../include/USAC/PUTSLAMEstimator.h"

//// COPIED FROM PUTSLAM:

//// THERE IS NOTHING LIKE THIS IN PUTSLAM

//// it's used in local optimization

// ============================================================================================
// generateRefinedModel: compute model using non-minimal set of samples
// default operation is to use a weight of 1 for every data point
// ============================================================================================
bool PUTSLAMEstimator::generateRefinedModel(std::vector<unsigned int>& sample,
	const unsigned int numPoints,
	bool weighted,
	double* weights)
{
	// MF:
	// both, the FundmatrixEstimator and the HomogEstimator are using the code below
	// TODO - this fragment should do something

	//// form the matrix of equations for this non-minimal sample
	//double *A = new double[numPoints * 9];
	//double *src_ptr;
	//double *dst_ptr = A;
	//for (unsigned int i = 0; i < numPoints; ++i)
	//{
	//	src_ptr = data_matrix_ + sample[i];
	//	for (unsigned int j = 0; j < 9; ++j)
	//	{
	//		if (!weighted)
	//		{
	//			*dst_ptr = *src_ptr;
	//		}
	//		else
	//		{
	//			*dst_ptr = (*src_ptr)*weights[i];
	//		}
	//		++dst_ptr;
	//		src_ptr += usac_num_data_points_;
	//	}
	//}

	//double Cv[9 * 9];
	//FTools::formCovMat(Cv, A, numPoints, 9);

	//double V[9 * 9], D[9], *p;
	//MathTools::svdu1v(D, Cv, 9, V, 9);

	//unsigned int j = 0;
	//for (unsigned int i = 1; i < 9; ++i)
	//{
	//	if (D[i] < D[j])
	//	{
	//		j = i;
	//	}
	//}
	//p = V + j;

	//for (unsigned int i = 0; i < 9; ++i)
	//{
	//	*(models_[0] + i) = *p;
	//	p += 9;
	//}
	//FTools::singulF(models_[0]);
	//// store denormalized version as well
	//double T2_F[9];
	//MathTools::mmul(T2_F, m_T2_trans_, models_[0], 3);
	//MathTools::mmul(models_denorm_[0], T2_F, m_T1_, 3);

	//delete[] A;

	return true;
}
