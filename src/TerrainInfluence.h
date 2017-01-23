#pragma once

#include "Simulation.h"
#include "Vector.h"
#include "ofxOpenCv.h"

class TerrainInfluence
{
	cv::Mat partialDerivative;
	int padding, rows, columns;
	Flocking* flock;
	vector<Boid>* boids;
	float weight;
public:
	TerrainInfluence(int _rows,int _columns,Flocking* _handle,float _weight=1, int _padding=1);
	void updateDepthImage(cv::Mat& depthImage);
	void updateBoids();
	void updateWeight(float _weight);
};