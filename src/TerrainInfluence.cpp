#include "TerrainInfluence.h"



void TerrainInfluence::init(int _rows, int _columns, Flocking* _handle, float _weight, int _padding,int _deltaHeight,float _mapRezImg,float _mapRezSim)
{
	padding = _padding;
	rows = _rows;
	columns = _columns;
	flock = _handle;
	boids = flock->getBoidsHandle();
	weight = _weight;
	deltaHeight = _deltaHeight;
	mapRezImg = _mapRezImg;
	mapRezSim = _mapRezSim;
	
	partialDerivative.create(rows, columns, CV_8UC3);
	partialDerivative.setTo(cv::Scalar(0,0));

}

void TerrainInfluence::updateDepthImage(cv::Mat& depthImage)
{
	for (int i = 0; i < depthImage.rows; i++)
	{
		int k = max(0, i - padding);
		cv::Vec3b* val = depthImage.ptr<cv::Vec3b>(i);
		cv::Vec3b* valBefore = depthImage.ptr<cv::Vec3b>(k);
		cv::Vec3b* derivative = partialDerivative.ptr<cv::Vec3b>(i);
		
		for (int j = 0; j < depthImage.cols; j++)
		{
			int delY=0, delX=0;
			int m = max(0, j - padding);
			delX = val[j][0] - val[m][0];
			delY = val[j][0] - valBefore[j][0];
			
			//derivative[j][0] = ofMap(delY,0,deltaHeight,0,255,true);
			//derivative[j][1] = ofMap(delY, -deltaHeight, 0, 255,0, true);
			
			derivative[j][0] = ofMap(delX, -deltaHeight, deltaHeight, 0, 255, true);
			derivative[j][1] = ofMap(delY, -deltaHeight, deltaHeight, 0, 255, true);
			derivative[j][2] = val[j][1];//Green Channel has edge walls. will use it for collisions. 
		}
		
	}
	
}

void TerrainInfluence::updateBoids()
{
	for (int i = 0; i < boids->size(); i++)
	{
		float x = (*boids)[i].loc.x*mapRezImg / mapRezSim;
		float y = (*boids)[i].loc.y*mapRezImg / mapRezSim;

		int x_ = (int)round(x);
		int y_ = (int)round(y);

		cv::Vec3b* derivative = partialDerivative.ptr<cv::Vec3b>(y_);
		bool hit = false;
		//Move the boids if collided with walls
		while (derivative[x_][2] == 255)
		{
			if (y_ < rows*0.5)
			{
				(*boids)[i].loc.y+=1;
				
			}
			else
			{
				(*boids)[i].loc.y -= 1;
			}
			y_ = (int)round((*boids)[i].loc.y*mapRezImg / mapRezSim);
			
			if (x_ < columns*0.5)
			{
				(*boids)[i].loc.x += 1;

			}
			else
			{
				(*boids)[i].loc.x -= 1;
			}
			x_ = (int)round((*boids)[i].loc.x*mapRezImg / mapRezSim);

			derivative = partialDerivative.ptr<cv::Vec3b>(y_);
			hit = true;
		}
		
		if(hit)
		(*boids)[i].vel *= -1;

		Vec2f gradient;
		gradient.x = ofMap(derivative[x_][0], 0, 255, -1, 1);
		gradient.y = ofMap(derivative[x_][1], 0, 255, -1, 1);

		(*boids)[i].seek(gradient + (*boids)[i].loc,weight, true);
		
	}
}

void TerrainInfluence::updateWeight(float _weight)
{
	weight = _weight;
}

cv::Mat * TerrainInfluence::getPartialDerivatives()
{
	return &partialDerivative;
}

void TerrainInfluence::updateDeltaHeight(int _deltaHeight)
{
	deltaHeight = _deltaHeight;
}

void TerrainInfluence::updatePadding(int _padding)
{
	padding = _padding;
}
