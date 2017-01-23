#include "TerrainInfluence.h"

TerrainInfluence::TerrainInfluence(int _rows, int _columns, Flocking* _handle, float _weight , int _padding)
{
	padding = _padding;
	rows = _rows;
	columns = _columns;
	flock = _handle;
	boids = flock->getBoidsHandle();
	weight = _weight;

	partialDerivative.create(rows, columns, CV_8UC2);
	partialDerivative.setTo(cv::Scalar(0));
	
}

void TerrainInfluence::updateDepthImage(cv::Mat& depthImage)
{
	for (int i = padding; i < depthImage.rows-padding; i++)
	{
		cv::Vec3b* val = depthImage.ptr<cv::Vec3b>(i);
		cv::Vec3b* valBefore = depthImage.ptr<cv::Vec3b>(i-padding);
		cv::Vec2b* derivative = partialDerivative.ptr<cv::Vec2b>(i);
		for (int j = padding; j < depthImage.cols-padding; j++)
		{
			float delY=0, delX=0;
			
			delX = val[j][0] - val[j-padding][0];
			delY = val[j][0] - valBefore[j][0];

			derivative[j][0] = delX;
			derivative[j][1] = delY;
		}
		
	}
	
}

void TerrainInfluence::updateBoids()
{
	for (int i = 0; i < boids->size(); i++)
	{
		float x = (*boids)[i].loc.x;
		float y = (*boids)[i].loc.y;

		

		cv::Vec2b* derivative = partialDerivative.ptr<cv::Vec2b>((int)y);
		
		Vec2f gradient;
		gradient.x = derivative[(int)x][0];
		gradient.y = derivative[(int)x][1];

		(*boids)[i].seek(gradient,weight, false);
		
	}
}

void TerrainInfluence::updateWeight(float _weight)
{
	weight = _weight;
}