#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){

	primaryScreenWidth = 1366;
	projectorWidth = 1920.0 *0.125;
	projectorHeight = 1080.0 * 0.125;

	projScaleX = 8;
	projScaleY = 8;


	window_width = projectorWidth*projScaleX + primaryScreenWidth;
	window_height = projectorHeight*projScaleY;
	
	//Adjust Counter
	kinectPointsCtr = projPointsCtr = 0;

	gui.setup("Controls","kinect_settings.xml");
	
	//Spectrum Adjust
	gui.add(minthreshold.setup("MINTHRESHOLD", 600, 0, 4500));
	gui.add(maxthreshold.setup("MAXTHRESHOLD",1200, 0, 4500));
	
	//Points selected on the Depth Image- RGB Scale.
	gui.add(kinectPoints[0].set("DEPTH FIRST", ofPoint(0, 0)));
	gui.add(kinectPoints[1].set("DEPTH SECOND", ofPoint(0, 0)));
	gui.add(kinectPoints[2].set("DEPTH THIRD", ofPoint(0, 0)));
	gui.add(kinectPoints[3].set("DEPTH FOURTH", ofPoint(0, 0)));
	
	//Points Projected to the Homo space. 
	gui.add(projPoints[0].set("HOMO FIRST", ofPoint(0, 0)));
	gui.add(projPoints[1].set("HOMO SECOND", ofPoint(0, 0)));
	gui.add(projPoints[2].set("HOMO THIRD", ofPoint(0, 0)));
	gui.add(projPoints[3].set("HOMO FOURTH", ofPoint(0, 0)));
	
	//True or False
	gui.add(Calibration.set("Calibration", Calibration));

    gui.loadFromFile("kinect_settings.xml");
	
	//Kinect Resources To use
	kinect.open();
	kinect.initDepthSource();
	
	ofSetWindowShape(window_width, window_height);

	//Kinect Depth image Adjust
	depthOffset.x = 400;
	depthOffset.y = 230;
	depthScale = 1;

    //Projector Display Adjust
	projOffset.x = 1366;
	projOffset.y = 0;

	
	totalPolygonPoints = 4;

	//depthImg is of OfImage type
	depthImg.allocate(512, 424, OF_IMAGE_COLOR);

	//depthImgCv is of ofxCvColorImage type
	depthImgCV.allocate(512, 424);

	//projectionImg is of ofxCvColorImage type
	//Image projected by the Projector
	projectionImg.allocate(projectorWidth, projectorHeight);
    
	//
	maskImg.allocate(projectorWidth, projectorHeight);
	
	//Mat Type
	processImg = cv::Mat(projectionImg.getCvImage());
	depthImgMat = cv::Mat(depthImgCV.getCvImage());
	maskImgMat = cv::Mat(maskImg.getCvImage());


	
	//Points already saved are passed onto the function to alot the x,y values to the KINECTPOLYGON and DEPTHPOLYGON
	for (int j=0;j<totalPolygonPoints;j++)
	{
		PointTransfer(kinectPoints[j], projPoints[j]);
		
		//Homography Matrix
		srcArray.push_back(depthPolygon[j]);
		dstArray.push_back(projPolygon[j]);
	}
	

}

//--------------------------------------------------------------
void ofApp::update() {
	
	kinect.update();
	if (kinect.isFrameNew()) {
		
		//to access the depth cam
		auto depth = kinect.getDepthSource();
		//depthPixel gets the depth value data for x and y
		auto depthPixel = depth->getPixels().getData();

		//accessing every DEPTH PIXEL on the X and the Y
		for (int x = 0;x < depth->getWidth(); x++) {
			for (int y = 0; y < depth->getHeight(); y++) {
                //Array representation on a single plane= to access data stored at x,y 
				int set = x + y*depth->getWidth();
				//extract data stored at set(x,y) and hand over to pixeldataXY
				int pixeldataXY = depthPixel[set];
				//pixeldataXY passed as input value. Only between Minthreshold and Maxthreshold I want data extraction 
				//Then map that value between 0 and 1, clamping set to true, set false for blackness beyond blue spec
				float a = ofMap(pixeldataXY, minthreshold, maxthreshold, 0, 1, true);
				
				ofColor pixels;
				//Spectrum Formation
				float red = 0,blue = 0,green = 0;
				if (a > 0 && a < 0.5) {
					 red = ofMap(a, 0, 0.5, 1, 0, true);
					 green = ofMap(a, 0, 0.5, 0,1, true);
					 blue = 0;
				}
				else if (a>=0.5 && a<1){
					 red = 0;
					 green = ofMap(a, 0.5, 1, 1, 0, true);
					 blue = ofMap(a, 0.5, 1, 0, 1, true);
				}
				//GUI HERE
				pixels.r = red*255;
				pixels.g = green*255;
				pixels.b = blue*255;

				depthImg.setColor(x, y, pixels);
				
			}
		}
		
		depthImg.update();
		
	}

	//ofxCvColorImage = ofImage 
	depthImgCV = depthImg;
	//Background of Projector 0
	projectionImg.set(0);
	//
	maskImg.set(0);


	cv::fillConvexPoly(maskImgMat, projPolygon, totalPolygonPoints,cv::Scalar(255,255,255));
	//cv::fillConvexPoly(depthImgMat, depthPolygon, totalPolygonPoints, cv::Scalar(0, 255, 0));
	
	if (Calibration)
	{
		homographyMatrix = cv::findHomography(srcArray,dstArray);
		cv::warpPerspective(depthImgMat, processImg, homographyMatrix, processImg.size());
		cv::bitwise_and(processImg, maskImgMat, processImg);
		
	}
	
	
}

//--------------------------------------------------------------
void ofApp::draw(){
	
	//Image diplayed
	depthImgCV.draw(depthOffset.x,depthOffset.y,depthImg.getWidth()*depthScale, depthImg.getHeight()*depthScale);
	gui.draw();

	if (Calibration)
	{
		projectionImg.draw(projOffset.x, projOffset.y, projectorWidth * projScaleX, projectorHeight * projScaleY);
	}
	else
	{
		maskImg.draw(projOffset.x, projOffset.y, projectorWidth * projScaleX, projectorHeight * projScaleY);

		for (int n = 0; n < totalPolygonPoints; n++)
		{
			//Points plotted displayed as Circle 
			ofSetColor(ofColor(0, 255, 0));
			ofDrawCircle(kinectPoints[n].get().x, kinectPoints[n].get().y, 3);
			ofDrawBitmapString(ofToString(n), kinectPoints[n].get().x, kinectPoints[n].get().y);
			
			//Points plotted displayed as Circle 
			ofSetColor(ofColor(255, 0, 0));
			ofDrawCircle(projPoints[n].get().x, projPoints[n].get().y, 3);
			ofDrawBitmapString(ofToString(n), projPoints[n].get().x, projPoints[n].get().y);
		}

		ofSetColor(ofColor(255, 255, 255));
	}
	    
	

	

}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
	
	if (key == 'k')
	{
		cout << "\n\n kinect points: \n";
		for (int n = 0; n <totalPolygonPoints; n++)
		{
			cout << " " << kinectPoints[n];
		}
		cout << "\n polygon points: \n";
		
		for (int n = 0; n <totalPolygonPoints; n++)
		{
			cout << " " << depthPolygon[n];
		}
		cout << "\n";
	}

	else if (key == 'c')
	{
        
		for (int n = 0; n <totalPolygonPoints; n++)
		{
			srcArray.push_back(depthPolygon[n]);
			dstArray.push_back(projPolygon[n]);
		}
		Calibration = true;
	}

	else if (key == 'C')
	{
		srcArray.clear();
		dstArray.clear();
		Calibration = false;
	}
	
	else if (key == 'd')
	{
		cout << "\n\tHomography : " << homographyMatrix << "\n";	
	}
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){
	
	if (button == OF_MOUSE_BUTTON_LEFT)
	{
		if (kinectPointsCtr >= totalPolygonPoints)
		{
			kinectPointsCtr = 0;
		}
		
		//Points we get on the plane = mousex and mousey
		kinectPoints[kinectPointsCtr].set(ofPoint(x, y));
		
		//mouse Points clicked minus Offset divided by scale= this gives us the exact point on the displayed image plane
		cv::Point polyPoint((kinectPoints[kinectPointsCtr].get().x - depthOffset.x)/depthScale, (kinectPoints[kinectPointsCtr].get().y - depthOffset.y) / depthScale);
		
		//depthPolygon is an array of cvpoint type
		depthPolygon[kinectPointsCtr] = polyPoint;

		kinectPointsCtr++;
	
	}

	else if (button == OF_MOUSE_BUTTON_RIGHT)
	{
		if (projPointsCtr >= totalPolygonPoints)
		{
			projPointsCtr = 0;
		}
		//Points we get on the plane = mousex and mousey
		projPoints[projPointsCtr].set(ofPoint(x, y));

		//mouse Points clicked minus Offset divided by scale= this gives us the exact point on the displayed image plane
		cv::Point polyPoint((projPoints[projPointsCtr].get().x - projOffset.x) / projScaleX, (projPoints[projPointsCtr].get().y - projOffset.y) / projScaleY);
		
		//projPolygon is an array of cvpoint type
		projPolygon[projPointsCtr] = polyPoint;

		projPointsCtr++;

	}
}

void ofApp::PointTransfer(ofParameter <ofPoint> polygonFrom, ofParameter<ofPoint> polygonTo) 
{
	
		cv::Point polyPoint((polygonFrom.get().x - depthOffset.x) / depthScale, (polygonFrom.get().y - depthOffset.y) / depthScale);
		depthPolygon[kinectPointsCtr] = polyPoint;

		cv::Point polyPoint1((polygonTo.get().x - projOffset.x) / projScaleX, (polygonTo.get().y - projOffset.y) / projScaleY);
		projPolygon[projPointsCtr] = polyPoint1;


		kinectPointsCtr++;
		projPointsCtr++;
	
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}

