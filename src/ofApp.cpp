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
	Calibration.addListener(this, &ofApp::calibrationButton);
    gui.loadFromFile("kinect_settings.xml");
	
	//Kinect Resources To use
	kinect.open();
	kinect.initDepthSource();
	
	ofSetWindowShape(window_width, window_height);

	//Kinect Depth image Adjust
	depthOffset.x = 500;
	depthOffset.y = 0;
	depthScale = 1.5;

    //Projector Display Adjust
	projOffset.x = 1366;
	projOffset.y = 0;

	
	totalPolygonPoints = 4;

	//depthImg is of OfImage type
	depthImg.allocate(512, 424, OF_IMAGE_COLOR);
	depthNormalized.allocate(512, 424, OF_IMAGE_GRAYSCALE);

	//depthImgCv is of ofxCvColorImage type
	depthImgCV.allocate(512, 424);
	depthNormalizedCV.allocate(512, 424);
	//projectionImg is of ofxCvColorImage type
	//Image projected by the Projector
	projectionImg.allocate(projectorWidth, projectorHeight);
	projectionNormalizedCV.allocate(projectorWidth, projectorHeight);
	projectionNormalizedCV.set(0, 0, 0);
	//
	maskImg.allocate(projectorWidth, projectorHeight);
	
	//Mat Type
	processImg = cv::Mat(projectionImg.getCvImage());
	depthImgMat = cv::Mat(depthImgCV.getCvImage());
	maskImgMat = cv::Mat(maskImg.getCvImage());
	depthNormalizedMat = cv::Mat(depthNormalizedCV.getCvImage());
	projectionNormalized = cv::Mat(projectionNormalizedCV.getCvImage());

	//Points already saved are passed onto the function to alot the x,y values to the KINECTPOLYGON and DEPTHPOLYGON
	for (int j=0;j<totalPolygonPoints;j++)
	{
		PointTransfer(kinectPoints[j], projPoints[j]);
		
		//Homography Matrix
		srcArray.push_back(depthPolygon[j]);
		dstArray.push_back(projPolygon[j]);
	}
	
	if(Calibration)
	homographyMatrix = cv::findHomography(srcArray, dstArray);

	//Simulation Variables setup
	mapRezSim = 1;
	mapRezImg = 1;
	maskPoints = 0;
	fishCount = 15;
	boundaryPadding = 10;
	collisionWeight = 0;
	startPosx = startPosy = 100;
	endPosx = projectorWidth * projScaleX * mapRezSim * 0.8;
	endPosy = projectorHeight * projScaleY * mapRezSim * 0.8;

	//Simulation Gui Setup
	boidScale.addListener(this, &ofApp::boidTriangleScaleChanged);
	simControls.setup("Simulation Controls", "simulation_settings.xml", 250, 10);
	simControls.add(maxSpeed.set("Max Speed", 2, 0, 10));
	simControls.add(maxForce.set("Max Force", 1, 0, 10));
	simControls.add(destWeight.set("Goal Weight", 0.1, 0, 1));
	simControls.add(flockSeparationWeight.set("Separation Weight", 1, 0, 2));
	simControls.add(flockAlignmentWeight.set("Alignment Weight", 0.5, 0, 2));
	simControls.add(flockCohesionWeight.set("Cohesion Weight", 0.25, 0, 2));
	simControls.add(flockSeparationRadius.set("Separation Radius", 15, 0, 50));
	simControls.add(flockAlignmentRadius.set("Alignment Radius", 20, 0, 50));
	simControls.add(flockCohesionRadius.set("Cohesion Radius", 30, 0, 50));
	simControls.add(startRadius.set("Start Radius", 50, 0, 200));
	simControls.add(endRadius.set("End Radius", 50, 0, 200));
	simControls.add(sleepTime.set("Sim Sleep Time", 0, 0, 0.1));
	simControls.add(randSeed.set("Random Seed", 0, 0, 1));
	simControls.add(boidScale.set("Boid Scale", 2, 0, 4));
	simControls.add(terrainWeight.set("Terrain Weight", 1, 0, 3));
	simControls.loadFromFile("simulation_settings.xml");
	//Simulation Gui Setup
	boidScale.addListener(this, &ofApp::boidTriangleScaleChanged);
	destWeight.addListener(this, &ofApp::simParamChanged);
	randSeed.addListener(this, &ofApp::simParamChanged);
	sleepTime.addListener(this, &ofApp::simParamChanged);
	maxSpeed.addListener(this, &ofApp::simParamChanged);
	maxForce.addListener(this, &ofApp::simParamChanged);
	flockSeparationWeight.addListener(this, &ofApp::simParamChanged);
	flockAlignmentWeight.addListener(this, &ofApp::simParamChanged);
	flockCohesionWeight.addListener(this, &ofApp::simParamChanged);
	flockSeparationRadius.addListener(this, &ofApp::simParamChanged);
	flockAlignmentRadius.addListener(this, &ofApp::simParamChanged);
	flockCohesionRadius.addListener(this, &ofApp::simParamChanged);
	startRadius.addListener(this, &ofApp::simParamStartRadiusChanged);
	endRadius.addListener(this, &ofApp::simParamEndRadiusChanged);
	terrainWeight.addListener(this, &ofApp::terrainWeightChanged);
	//Simulation initializaion
	sim.loadScene(startPosx, startPosy, endPosx, endPosy, projectorWidth * projScaleX * mapRezSim, projectorHeight * projScaleY * mapRezSim);
	sim.init(
		fishCount,//fish count
		destWeight,//destination Weight
		randSeed,//rand seed
		sleepTime,//sleep time
		boundaryPadding,//boundary padding
		maxSpeed,//max speed
		maxForce,//max force
		flockSeparationWeight,//flock separation weight
		flockAlignmentWeight,//flock alignment weight
		flockCohesionWeight,//flock cohesion weight
		collisionWeight,//collision weight
		flockSeparationRadius,//flock separation radius
		flockAlignmentRadius,//flock alignment radius
		flockCohesionRadius,//flock cohesion radius
		startRadius,//start position radius
		endRadius                    //end position radius
		);
	//Simulation Handels to access data if needed
	flock = sim.getFlockHandle();
	boids = flock->getBoidsHandle();

	terrain.init(projectorWidth,projectorHeight,flock,terrainWeight.get(),1);

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
				depthNormalized.setColor(x, y, ofColor(a*255));
				
			}
		}
		
		depthImg.update();
		depthNormalized.update();
		
	}

	//ofxCvColorImage = ofImage 
	depthImgCV = depthImg;
	//Background of Projector 0
	projectionImg.set(0);
	//
	maskImg.set(0);
	
	//Normalized depth image 0<--->255 range
	depthNormalizedCV = depthNormalized;
	
	cv::fillConvexPoly(maskImgMat, projPolygon, totalPolygonPoints,cv::Scalar(255,255,255));
	//cv::fillConvexPoly(depthImgMat, depthPolygon, totalPolygonPoints, cv::Scalar(0, 255, 0));
	
	if (Calibration)
	{
		
		cv::warpPerspective(depthImgMat, processImg, homographyMatrix, processImg.size());
		cv::bitwise_and(processImg, maskImgMat, processImg);
		
		cv::warpPerspective(depthNormalizedMat, projectionNormalized, homographyMatrix, projectionNormalized.size());

		terrain.updateDepthImage(projectionNormalized);
	}
	
	

	
}

//--------------------------------------------------------------
void ofApp::draw(){
	
	//Image diplayed
	depthImgCV.draw(depthOffset.x,depthOffset.y,depthImg.getWidth()*depthScale, depthImg.getHeight()*depthScale);
	gui.draw();
	simControls.draw();

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
	    
	
	//Flocksing Simulation update
	if (Calibration)
	{
		if (!sim.frame())
		{
			cout << "\nAdding new boids\n";
			sim.addAllBoids();
		}

		terrain.updateBoids();
		for (int i = 0; i < boids->size(); i++)
		{
			float x = (*boids)[i].loc.x*mapRezImg / mapRezSim;
			float y = (*boids)[i].loc.y*mapRezImg / mapRezSim;


			ofPoint boidPts[3];
			float angle = (*boids)[i].orient / 180.0*PI;
			ofColor randomCol(randomRange(100, 255, (*boids)[i].id * 123), randomRange(100, 255, (*boids)[i].id * 157), randomRange(100, 255, (*boids)[i].id * 921));

			ofFill();
			ofSetColor(randomCol);
			ofSetPolyMode(OF_POLY_WINDING_ODD);
			ofBeginShape();
			for (int i = 0; i < 3; i++)
			{
				boidPts[i].x = trianglePts[i].x*cos(angle) - trianglePts[i].y*sin(angle) + x;
				boidPts[i].y = trianglePts[i].y*cos(angle) + trianglePts[i].x*sin(angle) + y;
				//boidPts[i] += cv::Point(x, y);
				ofVertex(boidPts[i].x/*+primaryScreenWidth*/, boidPts[i].y);
			}
			ofEndShape();
			ofSetHexColor(0xffffff);


			//cv::Scalar randomCol(randomRange(50, 255, (*boids)[i].id * 123), randomRange(50, 255, (*boids)[i].id * 157), randomRange(50, 255, (*boids)[i].id * 921));
			//cv::fillConvexPoly(projectImgRGB, boidPts, 3, randomCol);

		}
		//Start
		//cv::circle(projectImgRGB, cv::Point((startPosx / mapRezSim)*mapRezImg, (startPosy / mapRezSim)*mapRezImg), (startRadius / mapRezSim)*mapRezImg, blue, 1);
		//Destination
		//cv::circle(projectImgRGB, cv::Point((endPosx / mapRezSim)*mapRezImg, (endPosy / mapRezSim)*mapRezImg), (endRadius / mapRezSim)*mapRezImg, yellow, 1);

	}//Sim update end
	

}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
	
	switch (key)
	{
	case OF_KEY_ESC:
		
		break;
	case 'k':
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
		break;

	case 'h':
		
		break;
	case 'a':
		cout << "\nAdding new boids\n";
		sim.addAllBoids();
		break;

	case 'r':
		cout << "\nRemove all boids\n";
		sim.removeAllBoids();

		break;

	case 'c':
		for (int n = 0; n <totalPolygonPoints; n++)
		{
			srcArray.push_back(depthPolygon[n]);
			dstArray.push_back(projPolygon[n]);
		}
		Calibration = true;

		break;

	case 'C':
		srcArray.clear();
		dstArray.clear();
		Calibration = false;

		break;
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
	
	if (!Calibration)
	{
		if (button == OF_MOUSE_BUTTON_LEFT)
		{
			if (kinectPointsCtr >= totalPolygonPoints)
			{
				kinectPointsCtr = 0;
			}

			//Points we get on the plane = mousex and mousey
			kinectPoints[kinectPointsCtr].set(ofPoint(x, y));

			//mouse Points clicked minus Offset divided by scale= this gives us the exact point on the displayed image plane
			cv::Point polyPoint((kinectPoints[kinectPointsCtr].get().x - depthOffset.x) / depthScale, (kinectPoints[kinectPointsCtr].get().y - depthOffset.y) / depthScale);

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
	else
	{
		if (button == OF_MOUSE_BUTTON_LEFT)
		{
			startPosx = x-primaryScreenWidth;
			startPosy = y;
			sim.setStart(startPosx, startPosy);
		}
		else if (button == OF_MOUSE_BUTTON_RIGHT)
		{
			endPosx = x- primaryScreenWidth;
			endPosy = y;
			sim.setDestination(endPosx, endPosy);
		}
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

void ofApp::calibrationButton(bool& val)
{
	if (Calibration)
	{
		for (int n = 0; n < totalPolygonPoints; n++)
		{
			srcArray.push_back(depthPolygon[n]);
			dstArray.push_back(projPolygon[n]);
		}
		homographyMatrix = cv::findHomography(srcArray, dstArray);
	}

	else
	{
		
		srcArray.clear();
		dstArray.clear();
		
	}

}



//Simulation Functions
void ofApp::simParamChanged(float& val)
{
	sim.updateSimParams(
		destWeight,//destination Weight
		randSeed,//rand seed
		sleepTime,//sleep time
		boundaryPadding,//boundary padding
		maxSpeed,//max speed
		maxForce,//max force
		flockSeparationWeight,//flock separation weight
		flockAlignmentWeight,//flock alignment weight
		flockCohesionWeight,//flock cohesion weight
		collisionWeight,//collision weight
		flockSeparationRadius,//flock separation radius
		flockAlignmentRadius,//flock alignment radius
		flockCohesionRadius,//flock cohesion radius
		startRadius,//start position radius
		endRadius); //end position radius);
}
void ofApp::simParamStartRadiusChanged(float& val)
{
	sim.setStart(startPosx, startPosy, startRadius);
}
void ofApp::simParamEndRadiusChanged(float& val)
{
	sim.setDestination(endPosx, endPosy, endRadius);
}
void ofApp::boidTriangleScaleChanged(float& val)
{
	trianglePts[0] = cv::Point(val*2.5 - val*0.5, 0);
	trianglePts[1] = cv::Point(-val - val*0.5, -val);
	trianglePts[2] = cv::Point(-val - val*0.5, val);
}
void ofApp::terrainWeightChanged(float& val)
{
	terrain.updateWeight(terrainWeight.get());
}
