#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){

	width = 1920 + 1366;
	height = 1080;
	
	kinectPointsCtr = projPointsCtr = 0;

	gui.setup("Controls","kinect_settings.xml");
	gui.add(minthreshold.setup("MINTHRESHOLD", 600, 0, 4500));
	gui.add(maxthreshold.setup("MAXTHRESHOLD",1200, 0, 4500));
	gui.loadFromFile("kinect_settings.xml");

	kinect.open();
	kinect.initDepthSource();
	//kinect.initColorSource();
	
	ofSetWindowShape(width, height);
		
	depthOffset.x = 400;
	depthOffset.y = 230;
	depthScale = 1;

	projectorWidth = 1920.0 *0.5;
	projectorHeight = 1080.0 * 0.5;
		
	projScaleX = 2;
	projScaleY = 2;

	projOffset.x = 1366;
	projOffset.y = 0;

	totalPolygonPoints = 4;

	calibrated = false;

	depthImg.allocate(512, 424, OF_IMAGE_COLOR);
	projectionImg.allocate(projectorWidth, projectorHeight);
	
	depthImgCV.allocate(512, 424);

	processImg = cv::Mat(projectionImg.getCvImage());
	depthImgMat = cv::Mat(depthImgCV.getCvImage());

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

				float a = ofMap(d, minthreshold, maxthreshold, 0, 255, true);

				ofColor pixels;
			
				float r1 = 0,b1 = 0,g1 = 0;
				if (a > 0 && a < 127) {

					 r1 = ofMap(a, 0, 127, 255, 0, true);
					 g1 = ofMap(a, 0, 127, 0, 255, true);
					 b1 = 0;
				}
				else if (a>=127 && a<256){

					 r1 = 0;
					 g1 = ofMap(a, 127, 255, 255, 0, true);
					 b1 = ofMap(a, 127, 255, 0, 255, true);
					
				}


				pixels.r = r1;
				pixels.g = g1;
				pixels.b = b1;

				depthImg.setColor(x, y, pixels);
				
			}
		}
		
		depthImg.update();
		
	}

	depthImgCV = depthImg;
	projectionImg.set(0);

	cv::fillConvexPoly(processImg, projPolygon, totalPolygonPoints,cv::Scalar(255,0,0));
	cv::fillConvexPoly(depthImgMat, depthPolygon, totalPolygonPoints, cv::Scalar(255, 0, 255));
	
	if (calibrated)
	{
		homographyMatrix = cv::findHomography(srcArray,dstArray);
		cv::warpPerspective(depthImgMat, processImg, homographyMatrix, processImg.size());
	}
	
	
}

//--------------------------------------------------------------
void ofApp::draw(){
	
	
	depthImgCV.draw(depthOffset.x,depthOffset.y,depthImg.getWidth()*depthScale, depthImg.getHeight()*depthScale);
	gui.draw();
	projectionImg.draw(projOffset.x, projOffset.y,projectorWidth * projScaleX,projectorHeight * projScaleY);
	    
	for (int n = 0; n < totalPolygonPoints; n++)
	{
		//ofSetColor(ofColor(255, 255, 0));
		ofDrawCircle(kinectPoints[n].x, kinectPoints[n].y,3);
		ofDrawBitmapString(ofToString(n), kinectPoints[n].x, kinectPoints[n].y);
		
		//ofSetColor(ofColor(0, 255, 255));
		ofDrawCircle(projPoints[n].x, projPoints[n].y, 3);
		ofDrawBitmapString(ofToString(n), projPoints[n].x, projPoints[n].y);
	 
	}

	//ofSetColor(ofColor(255, 255, 255));

	//cv::Mat my(sand);

	//cv::ellipse(sand, cv::Point(pt.x, pt.y), cv::Size(100.0, 160.0), 45, 0, 360, cv::Scalar(255, 0, 0), 1, 8);
	
	//imshow("Image", img2);
	
	/*
	ofSetColor(ofColor::white);

	ofBeginShape();
	ofRect(points.x, points.y, 10, 10);
	ofEndShape();
	*/

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
			//cv::Point pointsrc(kinectPoints[n].x, kinectPoints[n].y);
			srcArray.push_back(depthPolygon[n]);

			//cv::Vec2i pointdst(projPoints[n].x, projPoints[n].y);
			dstArray.push_back(projPolygon[n]);
			
		}
		calibrated = true;
	}

	else if (key == 'C')
	{
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

		kinectPoints[kinectPointsCtr] = cvPoint(x, y);
		
		cv::Point polyPoint((kinectPoints[kinectPointsCtr].x - depthOffset.x)/depthScale, (kinectPoints[kinectPointsCtr].y - depthOffset.y) / depthScale);
		depthPolygon[kinectPointsCtr] = polyPoint;
		
		kinectPointsCtr++;
		



	}

	else if (button == OF_MOUSE_BUTTON_RIGHT)
	{

		if (projPointsCtr >= totalPolygonPoints)
		{

			projPointsCtr = 0;

		}

		projPoints[projPointsCtr] = cvPoint(x, y);

		cv::Point polyPoint((projPoints[projPointsCtr].x - projOffset.x) / projScaleX, (projPoints[projPointsCtr].y - projOffset.y) / projScaleY);
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
