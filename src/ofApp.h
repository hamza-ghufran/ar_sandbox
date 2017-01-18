#pragma once

#include "ofMain.h"
#include "ofxKinectForWindows2.h"
#include "ofxGui.h"
#include "ofxOpenCv.h"



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

		ofxKFW2::Device kinect;

		ofxFloatSlider minthreshold;
		ofxFloatSlider maxthreshold;
		ofxPanel gui;


		ofImage depthImg;
	    ofxCvColorImage sand;
		cv::Mat processImg; 
		

		int width; 
		int height; 
		int kinectPointsCtr;
		int projPointsCtr;

		cv::Point kinectPoints[4];
		cv::Point projPoints[4];

};
