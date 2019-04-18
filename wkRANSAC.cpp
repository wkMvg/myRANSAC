#include <iostream>
#include <Eigen/Dense>
#include <vector>
using namespace std;
using namespace Eigen;

// @param dataSize ���������ݼ� 0~dataSize-1
// @param sampleSize ���������ݵ�����
// @param sampleId ���ղ����Ľ��
void randSample(int dataSize, int sampleSize, vector<int>& sampleId)
{
	if (dataSize < sampleSize)
		std::cerr << "the sample is not suitable\n";

	float sampleSize_still = sampleSize;
	float dataSize_still = dataSize;

	for (int i = 0; i < dataSize; i++)
	{
		/* ����һ��0-1֮�������� */
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
	// ����x��y�Ķ�Ӧ��ϵ���һ��ֱ�ߣ��������ݼ�����һ����outlier����˲���RANSAC�㷨
	vector<Vector3d> data{ Vector3d(0,0,1),
		Vector3d(1,1,1), Vector3d(2,2,1), Vector3d(3,3,1),
		Vector3d(3,2,1), Vector3d(4,4,1), Vector3d(10,2,1) };

	// ģ�Ͳ��� ax + by + c = 0
	Vector3d bestModel;

	// �����Ϊ��ȷ���ݵļ���
	vector<Vector3d> bestInliers;

	double bestError = 1000;

	// ��Ϊ�������ϵ���ֵ
	double thresholdInline = 0.8;
	// ��Ϊ�����ֱ������Ҫ�����ȷ����������ֵ
	int thresholdNum = data.size() / 2;
	// ����������
	int iterNum = 20;
	// �ܹ���ϳ�ֱ��ģ�͵����ٵ����
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