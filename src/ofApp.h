#pragma once

#include "ofMain.h"
#include "ofxKinectForWindows2.h"
#include "ofxGui.h"
#include "ofxOpenCv.h"

//Simulation Header
#include "Simulation.h"
#include "TerrainInfluence.h"

class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
		
		void PointTransfer(ofParameter <ofPoint> polygonFrom, ofParameter<ofPoint> polygonTo);


		ofxKFW2::Device kinect;

		ofxFloatSlider minthreshold;
		ofxFloatSlider maxthreshold;


		
		ofParameter <ofPoint> kinectPoints[4];
		ofParameter<ofPoint> projPoints[4];

		ofParameter <bool> Calibration;
		

		ofxPanel gui;
		void calibrationButton(bool& val);


		ofImage depthImg,depthNormalized;
		ofxCvColorImage depthImgCV, depthNormalizedCV;
		cv::Mat depthImgMat,depthNormalizedMat;
		
		//To process the image
		cv::Mat processImg; 
		ofxCvColorImage projectionImg, projectionNormalizedCV;
		cv::Mat projectionNormalized;

		//To Mask the projection
		ofxCvColorImage maskImg;
		cv::Mat maskImgMat;
		ofxCvColorImage maskImgNormalized;
		cv::Mat maskImgNormalizedMat;
		
		int primaryScreenWidth;
		int window_width; 
		int window_height; 
		int totalPolygonPoints;
		int kinectPointsCtr;
		int projPointsCtr;

		//cv::Point kinectPoints[4];
		//cv::Point projPoints[4];
		
		cv::Point depthOffset;
		float depthScale;
		
		float projScaleX,projScaleY;
		float projectorWidth;
		float projectorHeight;
		
		cv::Point projOffset;

		cv::Point depthPolygon[4];
		cv::Point projPolygon[4];
		
		cv::vector<cv::Point2f > srcArray;
		cv::vector<cv::Point2f > dstArray;

		cv::Mat homographyMatrix;
		
		//Flocking Simulation Variables and Functions
		void simParamChanged(float& val);
		void simParamStartRadiusChanged(float& val);
		void simParamEndRadiusChanged(float& val);
		void boidTriangleScaleChanged(float& val);
		void terrainWeightChanged(float& val);
		void terrainDeltaHeightChanged(int& val);
		void terrainEdgePaddingChanged(int& val);

		float mapRezSim;
		float mapRezImg;
		int maskPoints;
		int fishCount;
		int boundaryPadding;
		int collisionWeight;
		int startPosx, startPosy;
		int endPosx, endPosy;

		Simulation sim;
		Flocking* flock;
		vector<Boid>* boids;
		cv::Point trianglePts[3];

		ofxPanel simControls;
		ofxButton showContoursButton;
		ofParameter<float> maxSpeed;
		ofParameter<float> maxForce;
		ofParameter<float> destWeight;
		ofParameter<float> flockSeparationWeight;
		ofParameter<float> flockAlignmentWeight;
		ofParameter<float> flockCohesionWeight;
		ofParameter<float> flockSeparationRadius;
		ofParameter<float> flockAlignmentRadius;
		ofParameter<float> flockCohesionRadius;
		ofParameter<float> startRadius;
		ofParameter<float> endRadius;
		ofParameter<float> sleepTime;
		ofParameter<float> randSeed;
		ofParameter<float> boidScale;
		ofParameter<float> terrainWeight;
		ofParameter<int>	terrainDeltaHeight;
		ofParameter<int>	terrainEdgePadding;

		TerrainInfluence terrain;
		cv::Mat* partialDerivatives;
};
