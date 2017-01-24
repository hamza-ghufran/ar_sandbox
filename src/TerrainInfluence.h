#pragma once

#include "Flocking.h"
#include "ofxOpenCv.h"
#include "ofMain.h"

class TerrainInfluence
{
	cv::Mat partialDerivative;
	int padding, rows, columns;
	Flocking* flock;
	vector<Boid>* boids;
	float weight;
	int deltaHeight;
	float mapRezImg,mapRezSim;

public:
	//TerrainInfluence();
	void init(int _rows, int _columns, Flocking* _handle, float _weight = 1, int _padding = 1, int _deltaHeight = 30, float _mapRezImg=1, float _mapRezSim=1);
	void updateDepthImage(cv::Mat& depthImage);
	void updateBoids();
	void updateWeight(float _weight);
	cv::Mat* getPartialDerivatives();
	void updateDeltaHeight(int _deltaHeight);
	void updatePadding(int _padding);
};