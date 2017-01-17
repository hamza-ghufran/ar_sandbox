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


		
		ofImage img;
	    cv::Point pt;
		ofxCvColorImage sand;
		
		//vector<int> join;
		cv::Mat img1; 
		


		cv::vector<cv::Point> pointList;

		cv::Point kinectpoint[4];

};
