#include <iostream>
#include <Eigen/Dense>
#include <vector>
using namespace std;
using namespace Eigen;

// @param dataSize 待采样数据集 0~dataSize-1
// @param sampleSize 待采样数据的数量
// @param sampleId 最终采样的结果
void randSample(int dataSize, int sampleSize, vector<int>& sampleId)
{
	if (dataSize < sampleSize)
		std::cerr << "the sample is not suitable\n";

	float sampleSize_still = sampleSize;
	float dataSize_still = dataSize;

	for (int i = 0; i < dataSize; i++)
	{
		/* 生成一个0-1之间的随机数 */
		double randData = rand() / double(RAND_MAX);

		if (randData <= sampleSize_still / dataSize_still)
		{
			sampleId.push_back(i);

			if (sampleId.size() == sampleSize)
				break;

			sampleSize_still--;
		}
		dataSize_still--;
	}
}

double dist_point2line(Vector3d point, Vector3d lineModel)
{
	return abs(lineModel.x() * point.x() + lineModel.y() * point.y() + lineModel.z())
		/ sqrtf(lineModel.x() * lineModel.x() + lineModel.y() * lineModel.y());
}

int main()
{
	// 利用x和y的对应关系拟合一条直线，但是数据集中有一定的outlier，因此采用RANSAC算法
	vector<Vector3d> data{ Vector3d(0,0,1),
		Vector3d(1,1,1), Vector3d(2,2,1), Vector3d(3,3,1),
		Vector3d(3,2,1), Vector3d(4,4,1), Vector3d(10,2,1) };

	// 模型参数 ax + by + c = 0
	Vector3d bestModel;

	// 存放认为正确数据的集合
	vector<Vector3d> bestInliers;

	double bestError = 1000;

	// 认为点在线上的阈值
	double thresholdInline = 0.8;
	// 认为该拟合直线满足要求的正确点数量的阈值
	int thresholdNum = data.size() / 2;
	// 最大迭代次数
	int iterNum = 20;
	// 能够拟合出直线模型的最少点对数
	int minNum = 2;

	int iter = 0;

	while (iter < iterNum)
	{
		vector<int> randomId;
		randSample(data.size(), minNum, randomId);

		Vector3d currModle = data[randomId[0]].cross(data[randomId[1]]);
		vector<Vector3d> currInlier;

		int satisfyModelNum = 0;
		double totalError = 0;
		for (int i = 0; i < data.size(); i++)
		{
			if(i == randomId[0] || i == randomId[1])
				continue;
			double dist = dist_point2line(data[i], currModle);

			if (dist <= thresholdInline)
			{
				currInlier.push_back(data[i]);
				totalError += dist;
				satisfyModelNum++;
			}
		}

		if (satisfyModelNum >= thresholdNum)
		{
			if (totalError / satisfyModelNum < bestError)
			{
				bestModel = currModle;
				bestInliers = currInlier;
				bestError = totalError / satisfyModelNum;
			}
			else
			{
				iter++;
				continue;
			}
		}
		else
		{
			iter++;
			continue;
		}
		randomId.clear();
		iter++;
	}
}