#pragma once

#include "Vector.h"
#include <vector>

using namespace math;
using namespace std;

class Boid {
public:

	Boid(int id,int x, int y, int xbound, int ybound,
		int 	mboundaryPadding	,
		float 	mmaxSpeed 			,
		float 	mmaxForce 			,
		float 	mflockSepWeight 	,
		float 	mflockAliWeight 	,
		float 	mflockCohWeight 	,
		float 	mflockSepRadius 	,
		float 	mflockAliRadius 	,
		float 	mflockCohRadius 	);

    int id;

    float dist(Vec2f v1,Vec2f v2);
    float clamp(float val, float minval, float maxval);


	void update	(vector<Boid> &boids);

    void seek(Vec2f target,float weight=1);
    void avoid(Vec2f target,float weight=1);
    void boundCheck(int padding);

    void flock(vector<Boid> &boids);
    bool isHit(int x,int y, int radius);

	Vec2f steer(Vec2f target);

	Vec2f separate(vector<Boid> &boids);
	Vec2f align(vector<Boid> &boids);
	Vec2f cohesion(vector<Boid> &boids);

    void draw();

	Vec2f loc,vel,acc, prev_loc;
	Vec2i endCorner;

    float r;
	float orient;

	bool reachedDestination;
	bool hitObstacle;
	bool collided_with_contour=false;

	int 	boundaryPadding;
	float 	maxSpeed;
	float 	maxForce;
	float 	flockSepWeight;
	float 	flockAliWeight;
	float 	flockCohWeight;
	float 	flockSepRadius;
	float 	flockAliRadius;
	float 	flockCohRadius;

	void updateSimParams(
        float 	mmaxSpeed 			,
		float 	mmaxForce 			,
		float 	mflockSepWeight 	,
		float 	mflockAliWeight 	,
		float 	mflockCohWeight 	,
		float 	mflockSepRadius 	,
		float 	mflockAliRadius 	,
		float 	mflockCohRadius 	);

};

