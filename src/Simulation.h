#pragma once

#include <vector>
#include "Vector.h"
#include "Flocking.h"
#ifdef __unix__
# include <unistd.h>
#elif defined _WIN32
# include <windows.h>
#define sleep(x) Sleep(1000 * x)
#endif

using namespace math;

class Simulation
{
public:
	void loadScene(float startX=30,float startY=30,float endX=600,float endY=300,int _x_bound=640,int _y_bound=360);
	void init(

				int 	mfishCount 			,
				float 	mdestWeight 		,
				int 	mrandSeed 			,
				float	msleepTime			,
              	int 	mboundaryPadding 	,
              	float 	mmaxSpeed 			,
              	float 	mmaxForce 			,
           		float 	mflockSepWeight 	,
           		float 	mflockAliWeight 	,
           		float 	mflockCohWeight 	,
           		float 	mcollisionWeight 	,
           		float 	mflockSepRadius 	,
           		float 	mflockAliRadius 	,
           		float 	mflockCohRadius		,
				float	mstartPosRad		,
				float	mendPosRad
		);

	int frame();
	void run();
    void draw();
    void updateSimParams(
                float 	mdestWeight 		,
				int 	mrandSeed 			,
				float	msleepTime			,
              	int 	mboundaryPadding 	,
              	float 	mmaxSpeed 			,
              	float 	mmaxForce 			,
           		float 	mflockSepWeight 	,
           		float 	mflockAliWeight 	,
           		float 	mflockCohWeight 	,
           		float 	mcollisionWeight 	,
           		float 	mflockSepRadius 	,
           		float 	mflockAliRadius 	,
           		float 	mflockCohRadius		,
				float	mstartPosRad		,
				float	mendPosRad);
	Flocking* getFlockHandle();
	void setDestination(int x, int y);
	void setDestination(int x, int y,float radius);
	void setStart(int x, int y);
	void setStart(int x, int y,float radius);
	void addAllBoids();
	void removeAllBoids();


	Simulation()
	{

    }

private:


    Flocking flock;

	unsigned int x_bound;
    unsigned int y_bound;
    Vec2f startPosition;
	Vec2f endPosition;
	float startPositionRadius;
	float endPositionRadius;

	//init variables
  	int fishCount 			;
  	int boundaryPadding 	;
  	float maxSpeed 			;
  	float maxForce 			;
	float flockSepWeight 	;
	float flockAliWeight 	;
	float flockCohWeight 	;
    float collisionWeight 	;
	float flockSepRadius 	;
	float flockAliRadius 	;
	float flockCohRadius 	;
    float destWeight 		;
    int randSeed 			;
    float sleepTime;

};
