/** @file dbscan.cpp
 *
 * \brief The proposed clustering algorithm
 * \author Michal Nowicki
 *
 */
#include "../include/Matcher/dbscan.h"

#include <iostream>

DBScan::DBScan(double _eps, int _minPts, int _featuresFromCluster) {
	eps = _eps;
	minPts = _minPts;
	featuresFromCluster = _featuresFromCluster;
}

void DBScan::expandCluster(std::vector<int> neighbourList,
		int clusteringSetSize,
		int &C) {

	// testing the neighbours
	for (int j = 0; j < neighbourList.size(); j++) {
		int x = neighbourList[j];

		// If not visited
		if (visited[x] != true) {
			visited[x] = true;
			std::vector<int> neighbourNeighbourList;

			// Calculating the number of neighbours
			for (int k = 0; k < clusteringSetSize; k++) {
				if (!visited[k] && dist[std::min(x, k)][std::max(x, k)] < eps) {
					neighbourNeighbourList.push_back(k);
				}
			}

			// If it has enough neighbours it's neighbours can be checked
			// Merging ...
			if (neighbourNeighbourList.size() >= minPts) {
				neighbourList.insert(neighbourList.end(), neighbourNeighbourList.begin(), neighbourNeighbourList.end());
			}
		}

		// if it is not yet labeled
		if (cluster[x] == 0)
			cluster[x] = C;
	}
}

int DBScan::findingClusters(int clusteringSetSize) {
	// Starting cluster id
	int C = 1;

	// For all points
	for (int i = 0; i < clusteringSetSize; i++) {
		if (visited[i] != true) {
			visited[i] = true;
			std::vector<int> neighbourList;
			// Finding neighbours
			for (int k = 0; k < clusteringSetSize; k++) {
				if (dist[std::min(i, k)][std::max(i, k)] < eps) {
					neighbourList.push_back(k);
				}
			}
			// If there are not enough neighbours to form a cluster
			if (neighbourList.size() < minPts)
				cluster[i] = -1;
			else {
				// There is a need cluster!
				cluster[i] = C;
				expandCluster(neighbourList, clusteringSetSize, C);
				C++;
			}
		}
	}
	return C;
}

void DBScan::run(std::vector<cv::KeyPoint> & clusteringSet) {
	int clusteringSetSize = clusteringSet.size();

	// Calculating similarity matrix
	dist = std::vector<std::vector<float> >(clusteringSetSize,
			std::vector<float>(clusteringSetSize, 0));

	for (int i = 0; i < clusteringSetSize; i++)
		for (int j = i; j < clusteringSetSize; j++)
			dist[i][j] = cv::norm(clusteringSet[i].pt - clusteringSet[j].pt);

	// Preparation - visited nodes information
	visited = std::vector<bool>(clusteringSetSize, false);

	// Output information
	// -1 means noise
	// 0 not yet processed
	// >0 belongs to group of given id
	cluster = std::vector<int>(clusteringSetSize);

	// For all points
	int clusterCount = findingClusters(clusteringSetSize);

	// Just leave strongest from each cluster
	std::vector<int> clusterChosenCount(clusterCount);
	for (int i=0;i<clusteringSetSize;i++ ) {
		int clusterId = cluster[i];
		if ( clusterId > 0)
		{
			if (clusterChosenCount[clusterId] > featuresFromCluster - 1)
			{
				if (clusteringSet[i].octave == -5)
					std::cout
							<< "DBScan issue : SHIT :/ Octave == -5 -> need to find another way of marking to erase" << std::endl;
				clusteringSet[i].octave = -5;
			} else
				clusterChosenCount[clusterId]++;
		}
	}

	// Remove those bad elements
	clusteringSet.erase(
			std::remove_if(clusteringSet.begin(), clusteringSet.end(),
					[] (cv::KeyPoint &kp) {return kp.octave == -5;}), clusteringSet.end());
}
