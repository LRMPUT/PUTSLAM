/** @file dbscan.cpp
 *
 * \brief The proposed clustering algorithm
 * \author Michal Nowicki
 *
 */
#include "../include/Matcher/dbscan.h"

#include <iostream>

DBScan::DBScan(double _eps, int _minPts) {
	eps = _eps;
	minPts = _minPts;
}

void DBScan::expandCluster(std::vector<int> neighbourList, bool *visited,
		int clusteringSetSize, double **dist, std::vector<int> & cluster, int &C) {
	// testing the neighbours
	for (int j = 0; j < neighbourList.size(); j++) {
		int x = neighbourList[j];

		// If not visited
		if (visited[x] != 1) {
			visited[x] = 1;
			std::vector<int> neighbourNeighbourList;

			// Calculating the number of neighbours
			for (int k = 0; k < clusteringSetSize; k++) {
				if (dist[std::min(x, k)][std::max(x, k)] < eps) {
					neighbourNeighbourList.push_back(k);
				}
			}

			// If it has enough neighbours it's neighbours can be checked
			if (neighbourNeighbourList.size() >= minPts) {
				for (int g = 0; g < neighbourNeighbourList.size(); g++)
					neighbourList.push_back(neighbourNeighbourList[g]);
			}
		}

		// if it is not yet labeled
		if (cluster[x] == 0)
			cluster[x] = C;
	}
}



void DBScan::run(std::vector<cv::KeyPoint> & clusteringSet, std::vector<int> & cluster) {
	int clusteringSetSize = clusteringSet.size();


	// Let's have some fun with indexing
	std::vector<cv::Point2f> clusteringSetPts;
	cv::KeyPoint::convert(clusteringSet, clusteringSetPts);
	cv::Mat clusteringSetPtsMat = cv::Mat(clusteringSetPts).reshape(1);

	// Let's create an index structure
	std::cout<<"samples size : " << clusteringSetPtsMat.rows << " " << clusteringSetPtsMat.cols << std::endl;
	std::cout<<clusteringSetPtsMat.at<float>(0,0) << " " << clusteringSetPtsMat.at<float>(0,1) <<std::endl;
	std::cout<<clusteringSetPtsMat.at<float>(1,0) << " " << clusteringSetPtsMat.at<float>(1,1) <<std::endl;
	std::cout<<clusteringSetPtsMat.at<float>(2,0) << " " << clusteringSetPtsMat.at<float>(2,1) <<std::endl;
	cv::flann::Index flannStructure(clusteringSetPtsMat, cv::flann::KDTreeIndexParams(8), cvflann::FLANN_DIST_EUCLIDEAN);
	flannStructure.build(clusteringSetPtsMat, cv::flann::KDTreeIndexParams(8), cvflann::FLANN_DIST_EUCLIDEAN);

	// Ask a question
	std::vector<cv::Point2f> query;
	query.push_back(clusteringSet[0].pt);
	cv::Mat queryMat = cv::Mat(query).reshape(1);
	cv::Mat answer, distances;
	float radius = 80;
	std::cout << "query size : " << queryMat.rows << " " << queryMat.cols << std::endl;
	std::cout << "Query for : " << queryMat.at<float>(0,0) << " " << queryMat.at<float>(0,1) << std::endl;
	flannStructure.radiusSearch(queryMat, answer, distances, radius, clusteringSetPts.size(), cv::flann::SearchParams(64));

	std::cout<<"TYPES: " << answer.type() << " " << distances.type() << std::endl;
	for (int k = 0; k < 5; k++)
		std::cout << "Answer for " << k << ": " << answer.at<int>(k)
				<< " dist : " << sqrt(distances.at<float>(k)) << " vs "
				<< cv::norm(
						clusteringSet[0].pt
								- clusteringSet[answer.at<int>(k)].pt)<< std::endl;

	int compareFLANN = 0;
	for (int j = 0; j < distances.cols; j++)
	{
//		std::cout<<distances.at<float>(0,j) << " " <<std::endl;
		if (answer.at<int>(0,j) > 0)
			compareFLANN++;
	}
	std::cout << "Results from flann : Found " << compareFLANN << " Sizes: "
			<< answer.type() << " " <<answer.rows << " " << answer.cols << std::endl;

	int compare = 0;
	for (int j = 0; j < clusteringSetSize; j++)
		if ( cv::norm(clusteringSet[0].pt - clusteringSet[j].pt)  < 10 )
			compare++;

	std::cout<<"Results from brute : Found " << compare << std::endl;

	int K = 5;
	cv::Mat resps;//(1, K, CV_32F);
	cv::Mat nresps;//(1, K, CV_32S);
	cv::Mat dists;//(1, K, CV_32F);
	flannStructure.knnSearch(queryMat,nresps,dists,K,cv::flann::SearchParams(64));
	std::cout<<"TYPES: " << nresps.type() << " " << dists.type() << std::endl;
	for (int k=0;k<5;k++)
		std::cout<<"Answer for "<<k<<": " << nresps.at<int>(k) << " dist : " << dists.at<float>(k) << std::endl;

	int a;
	scanf("%d",&a);

	// Calculating similarity matrix
	double **dist;

	dist = new double *[clusteringSetSize];
	for (int i = 0; i < clusteringSetSize; i++)
		dist[i] = new double[clusteringSetSize];

	for (int i = 0; i < clusteringSetSize; i++)
		for (int j = i; j < clusteringSetSize; j++)
			dist[i][j] = cv::norm(clusteringSet[i].pt - clusteringSet[j].pt);

	// Preparation
	int C = 0;
	bool * visited = new bool[clusteringSetSize]();
	bool * noise = new bool[clusteringSetSize]();

	for (int i = 0; i < clusteringSetSize; i++)
		cluster.push_back(0);

	// For all points
	for (int i = 0; i < clusteringSetSize; i++) {
		if (visited[i] != true) {
			visited[i] = true;
			std::vector<int> neighbourList;

			// Finding neighbours
			for (int k = 0; k < clusteringSetSize; k++) {
				if (dist[std::min(i,k)][std::max(i,k)] < eps) {
					neighbourList.push_back(k);
				}
			}

			// If there are not enough neughbours to form a cluster
			if (neighbourList.size() < minPts)
				noise[i] = true;
			else {
				C++;

				// Test if the cluster can be expanded
				cluster[i] = C;

				expandCluster(neighbourList, visited, clusteringSetSize,
						dist, cluster, C);
			}
		}
	}

	// Clearing
	delete[] visited;
	delete[] noise;
	for (int i = 0; i < clusteringSetSize; i++)
		delete[] dist[i];
	delete[] dist;
}


