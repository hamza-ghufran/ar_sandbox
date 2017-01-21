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
	
	kinectPointsCtr = projPointsCtr = 0;

	gui.setup("Controls","kinect_settings.xml");
	gui.add(minthreshold.setup("MINTHRESHOLD", 600, 0, 4500));
	gui.add(maxthreshold.setup("MAXTHRESHOLD",1200, 0, 4500));
	

	gui.add(kinectPoints[0].set("DEPTH FIRST", ofPoint(0, 0)));
	gui.add(kinectPoints[1].set("DEPTH SECOND", ofPoint(0, 0)));
	gui.add(kinectPoints[2].set("DEPTH THIRD", ofPoint(0, 0)));
	gui.add(kinectPoints[3].set("DEPTH FOURTH", ofPoint(0, 0)));
	

	gui.add(projPoints[0].set("HOMO FIRST", ofPoint(0, 0)));
	gui.add(projPoints[1].set("HOMO SECOND", ofPoint(0, 0)));
	gui.add(projPoints[2].set("HOMO THIRD", ofPoint(0, 0)));
	gui.add(projPoints[3].set("HOMO FOURTH", ofPoint(0, 0)));
	


	gui.loadFromFile("kinect_settings.xml");
	//gui.add(TOGGLE HOMO POINTS)

	//gui.add(SAVE PROJECTION POINTS)
	//gui.add(TOGGLE PROJECTION POINTS


	kinect.open();
	kinect.initDepthSource();
	//kinect.initColorSource();
	
	ofSetWindowShape(window_width, window_height);
		
	depthOffset.x = 400;
	depthOffset.y = 230;
	depthScale = 1;

	

	projOffset.x = 1366;
	projOffset.y = 0;

	totalPolygonPoints = 4;

	calibrated = true;

	depthImg.allocate(512, 424, OF_IMAGE_COLOR);
	depthImgCV.allocate(512, 424);

	projectionImg.allocate(projectorWidth, projectorHeight);
	maskImg.allocate(projectorWidth, projectorHeight);
	

	processImg = cv::Mat(projectionImg.getCvImage());
	depthImgMat = cv::Mat(depthImgCV.getCvImage());
	maskImgMat = cv::Mat(maskImg.getCvImage());


	for (int j=0;j<totalPolygonPoints;j++)
	{
		cv::Point polyPoint((kinectPoints[j].get().x - depthOffset.x) / depthScale, (kinectPoints[j].get().y - depthOffset.y) / depthScale);
		depthPolygon[j] = polyPoint;

		cv::Point polyPoint1((projPoints[j].get().x - projOffset.x) / projScaleX, (projPoints[j].get().y - projOffset.y) / projScaleY);
		
		projPolygon[j] = polyPoint1;

		srcArray.push_back(depthPolygon[j]);
		dstArray.push_back(projPolygon[j]);
	}
	

}

//--------------------------------------------------------------
void ofApp::update() {
	
	kinect.update();
	if (kinect.isFrameNew()) {
		auto depth = kinect.getDepthSource();

		auto depthPixel = depth->getPixels().getData();

		
		for (int x = 0;x < depth->getWidth(); x++) {
			
			
			for (int y = 0; y < depth->getHeight(); y++) {
				
				
				int set = x + y*depth->getWidth();

				int d = depthPixel[set];

				float a = ofMap(d, minthreshold, maxthreshold, 0, 1, true);

				ofColor pixels;
			
				float r1 = 0,b1 = 0,g1 = 0;
				if (a > 0 && a < 0.5) {
				
					 r1 = ofMap(a, 0, 0.5, 1, 0, true);
					 g1 = ofMap(a, 0, 0.5, 0,1, true);
					 b1 = 0;
				}
			

				else if (a>=0.5 && a<1){

					 r1 = 0;
					 g1 = ofMap(a, 0.5, 1, 1, 0, true);
					 b1 = ofMap(a, 0.5, 1, 0, 1, true);
					
				}

				//GUI HERE
				pixels.r = r1*255;
				pixels.g = g1*255;
				pixels.b = b1*255;

				depthImg.setColor(x, y, pixels);
				
			}
		}
		
		depthImg.update();
		
	}

	depthImgCV = depthImg;
	projectionImg.set(0);
	maskImg.set(0);


	cv::fillConvexPoly(maskImgMat, projPolygon, totalPolygonPoints,cv::Scalar(255,255,255));
	//cv::fillConvexPoly(depthImgMat, depthPolygon, totalPolygonPoints, cv::Scalar(0, 255, 0));
	
	if (calibrated)
	{
		homographyMatrix = cv::findHomography(srcArray,dstArray);
		cv::warpPerspective(depthImgMat, processImg, homographyMatrix, processImg.size());
		cv::bitwise_and(processImg, maskImgMat, processImg);
		
	}
	
	
}

//--------------------------------------------------------------
void ofApp::draw(){
	
	
	depthImgCV.draw(depthOffset.x,depthOffset.y,depthImg.getWidth()*depthScale, depthImg.getHeight()*depthScale);
	gui.draw();

	if (calibrated)
	{
		projectionImg.draw(projOffset.x, projOffset.y, projectorWidth * projScaleX, projectorHeight * projScaleY);
	}
	else
	{
		maskImg.draw(projOffset.x, projOffset.y, projectorWidth * projScaleX, projectorHeight * projScaleY);

		

		for (int n = 0; n < totalPolygonPoints; n++)
		{
			//kinectPoints[n].set(ofColor(0, 255, 0));
			ofSetColor(ofColor(0, 255, 0));
			ofDrawCircle(kinectPoints[n].get().x, kinectPoints[n].get().y, 3);
			ofDrawBitmapString(ofToString(n), kinectPoints[n].get().x, kinectPoints[n].get().y);
			
			

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
        //GUI HERE
		for (int n = 0; n <totalPolygonPoints; n++)
		{
			srcArray.push_back(depthPolygon[n]);
			dstArray.push_back(projPolygon[n]);
			
		}


		calibrated = true;
	}

	else if (key == 'C')
	{
		//GUI HERE
		srcArray.clear();
		dstArray.clear();
		calibrated = false;
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
		
		kinectPoints[kinectPointsCtr].set(ofPoint(x, y));
		
		cv::Point polyPoint((kinectPoints[kinectPointsCtr].get().x - depthOffset.x)/depthScale, (kinectPoints[kinectPointsCtr].get().y - depthOffset.y) / depthScale);
		depthPolygon[kinectPointsCtr] = polyPoint;

	   
		kinectPointsCtr++;
		



	}

	else if (button == OF_MOUSE_BUTTON_RIGHT)
	{

		if (projPointsCtr >= totalPolygonPoints)
		{

			projPointsCtr = 0;

		}

		projPoints[projPointsCtr].set(ofPoint(x, y));

		cv::Point polyPoint((projPoints[projPointsCtr].get().x - projOffset.x) / projScaleX, (projPoints[projPointsCtr].get().y - projOffset.y) / projScaleY);
		
		//GUI HERE
		projPolygon[projPointsCtr] = polyPoint;

		projPointsCtr++;




	}
		


		
	
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
