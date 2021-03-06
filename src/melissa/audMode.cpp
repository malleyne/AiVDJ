#include "audMode.h"
#include "ofAppGlutWindow.h"

//--------------------------------------------------------------
void audMode::setup() {
	//ofSetLogLevel(OF_LOG_VERBOSE);
	
	/*----------------kinect setup--------------------*/
	// enable depth->video image calibration
	kinect.setRegistration(true);
	kinect.init();
	kinect.open();
	/*------------------------------------------------*/
	
	/*------------------set up gray image----------------*/
	grayImage.allocate(kinect.width, kinect.height);
	grayThreshNear.allocate(kinect.width, kinect.height);
	grayThreshFar.allocate(kinect.width, kinect.height);
	
	nearThreshold = 230;
	farThreshold = 70;
	/*------------------------------------------------*/
	
	/*------------------------------------------------*/
	ofSetFrameRate(60);
	// zero the tilt on startup
	angle = 0;
	kinect.setCameraTiltAngle(angle);
	// start from the front
	bDrawPointCloud = false;
	/*------------------------------------------------*/
	
	/*------------------arduino stuffs----------------*/
	serial.enumerateDevices();
	serial.setup("/dev/tty.usbmodem641",9600);
	/*------------------------------------------------*/
	
	updates = 0;
	choicecolor = ofColor(0,0,0);
	distance = 0.0;
	last_vec.set(0,0,0);
	moveCam = 0.0;
	flipBit = 1;
}

//--------------------------------------------------------------
void audMode::update(float vol) {
	
	ofBackground(0);
	
	kinect.update();
	
	// there is a new frame and we are connected
	if(kinect.isFrameNew()) {
		updates++;
		// load grayscale depth image from the kinect source
		grayImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
		
		grayThreshNear = grayImage;
		grayThreshFar = grayImage;
		grayThreshNear.threshold(nearThreshold, true);
		grayThreshFar.threshold(farThreshold);
		cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
		
		// update the cv images
		grayImage.flagImageChanged();
		
		// find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
		// also, find holes is set to true so we will get interior contours as well....
		contourFinder.findContours(grayImage, 10, (kinect.width*kinect.height)/2, 10, false);
		
		
		
		/*Now that the contours have been processed we can try to figure out which is the largest 
		 since that�s probably the one we�re interested in.*/
		int size = contourFinder.blobs.size();
		printf("contourFinder.blobs.size() = %d\n", int(contourFinder.blobs.size()));
		if(size > 0) {
			//why do we skip the last one? if doesn't work subtract 1 from condition portion of loop
			ofVec3f currV;
			currV.set(0,0,0);
			for (int i = 1; i < contourFinder.blobs.size(); i++) {
				currV += contourFinder.blobs.at(i).centroid;
			}
			currV /= size; //find average blob
			if (updates == 1) {
				last_vec = currV;
			} else {
				distance += last_vec.distance(currV); //get total distance travelled for certain num of updates
				//printf("distance %d", distance);
				last_vec = currV;
			}
			// this is important: we don't want to send data to the serial port
			// on every frame because the Arduino runs much more slowly than the
			// oF app.
			easyCam.begin();	
			if (moveCam > 800 || moveCam < -800) {
				flipBit*=-1;
			}
			moveCam+=(5*flipBit);
			float xx = moveCam;
			float yy=sin(ofGetElapsedTimef()*0.4)*150;
			float zz=cos(ofGetElapsedTimef()*0.4)*150;
			//easyCam.pan(180);
			easyCam.setPosition(xx,yy,zz);
			if(ofGetFrameNum() % 5 == 0) {
				//loudness of sound = brightness of lights that are on	
				//speed determines number lit
				if (updates >= 16) {
					//mod by 10 broken into 3 sections forgetting zero
					int speed = (int(distance*2 + 1)/updates)%10;
					printf("speed = %d\n", speed);
					//speed = 1;
					int value = (int) getValue(vol);
					switch (speed) {
						case 1: //red
							//serial.writeByte(255); //r
							serial.writeByte(value);
							serial.writeByte(0); //g
							serial.writeByte(0);//b
							choicecolor = ofColor(255,0,0, value);
							printf("red = %d\n", speed);
							break;
						case 2:
							choicecolor = ofColor(255,255,0, value);
						case 3: //yellow
							serial.writeByte(value);
							serial.writeByte(value);
							serial.writeByte(0);
							printf("yellow = %d\n", speed);
							choicecolor = ofColor(255,255,0, value);
							break;
						case 4:
							choicecolor = ofColor(0,255,0,value);
						case 5: //green
							serial.writeByte(0);
							serial.writeByte(value);
							serial.writeByte(0);
							choicecolor = ofColor(0,255,0, value);
							printf("green = %d\n", speed);
							break;
						case 6: //cyan
							serial.writeByte(0);
							serial.writeByte(value);
							serial.writeByte(value);
							choicecolor = ofColor(0,255,255,value);
							printf("cyan = %d\n", speed);
							break;
						case 7: //blue
							serial.writeByte(0);
							serial.writeByte(0);
							serial.writeByte(value);
							choicecolor = ofColor(0, 0,255,value);
							printf("blue = %d\n", speed);
							break;
						case 8: //magenta
							serial.writeByte(value);
							serial.writeByte(0);
							serial.writeByte(value);
							choicecolor = ofColor(255,0,255, value);
							printf("magenta = %d\n", speed);
							break;
						default: //white
							serial.writeByte(value);
							serial.writeByte(value);
							serial.writeByte(value);
							choicecolor = ofColor(255,255,255,value);
							printf("white = %d\n", speed);
							break;
					}					
					updates = 0;
					//serial.writeByte(contourFinder.blobs.at(largest).centroid.x/kinect.width * 255);
				}

				easyCam.end();
			}
		}
	}	
}

//--------------------------------------------------------------
void audMode::draw() {

	//ofBackground(0);
	ofBackground(10);
	//ofSetColor(0, 0, 0);
	easyCam.begin();
	drawPointCloud();
	// draw from the live kinect
	grayImage.draw(10, 320, 400, 300);
	contourFinder.draw(10, 320, 400, 300);
	easyCam.end();	
}

void audMode::drawPointCloud() {
	int w = 640;
	int h = 480;
	ofMesh mesh;
	mesh.setMode(OF_PRIMITIVE_POINTS);
	ofMesh mesh2;
	mesh2.setMode(OF_PRIMITIVE_POINTS);

	int step = 2;
	for(int y = 0; y < h; y += step) {
		for(int x = 0; x < w; x += step) {
			if(kinect.getDistanceAt(x, y) > 0) {
				mesh.addColor(ofColor(choicecolor));
				mesh.addVertex(kinect.getWorldCoordinateAt(x, y));
				mesh2.addColor(ofColor(choicecolor));
				mesh2.addVertex(kinect.getWorldCoordinateAt(x, y));
			}
		}
	}
	glPointSize(1);
	ofPushMatrix();
	// the projected points are 'upside down' and 'backwards' 
	ofScale(-1, -1, -1);
	ofTranslate(w/2, 0, -1000); // center the points a bit
	glEnable(GL_DEPTH_TEST);
	mesh.drawVertices();
	glDisable(GL_DEPTH_TEST);
	ofPopMatrix();
							  
	glPointSize(1);
	ofPushMatrix();
	// the projected points are 'upside down' and 'backwards' 
	ofScale(1, -1, -1);
	ofTranslate(-w/2, 0, -1000); // center the points a bit
	glEnable(GL_DEPTH_TEST);
	mesh2.drawVertices();
	glDisable(GL_DEPTH_TEST);
	ofPopMatrix();
}

//--------------------------------------------------------------
void audMode::exit() {
	kinect.setCameraTiltAngle(0); // zero the tilt on exit
	kinect.close();
}

//using volume to determine the brightness of the color
float audMode::getValue(float volume) {
	return mapValue(volume, 0, 2)+25;
}

//map value to 0-255 range from low-high range
float audMode::mapValue(float value, float low, float high) {
	float fraction = value / (high-low);
	return 255 * fraction;
}

//--------------------------------------------------------------
void audMode::AudkeyPressed (int key) {
	printf("i hit fucking p");
	switch (key) {
		case ' ':
			break;
			
		case'p':
			bDrawPointCloud = !bDrawPointCloud;
			break;
			
		case '>':
		case '.':
			farThreshold ++;
			if (farThreshold > 255) farThreshold = 255;
			break;
			
		case '<':
		case ',':
			farThreshold --;
			if (farThreshold < 0) farThreshold = 0;
			break;
			
		case '+':
		case '=':
			nearThreshold ++;
			if (nearThreshold > 255) nearThreshold = 255;
			break;
			
		case '-':
			nearThreshold --;
			if (nearThreshold < 0) nearThreshold = 0;
			break;
			
		case 'w':
			kinect.enableDepthNearValueWhite(!kinect.isDepthNearValueWhite());
			break;
			
		case 'o':
			kinect.setCameraTiltAngle(angle); // go back to prev tilt
			kinect.open();
			break;
			
		case 'c':
			kinect.setCameraTiltAngle(0); // zero the tilt
			kinect.close();
			break;
			
		case OF_KEY_UP:
			angle++;
			if(angle>30) angle=30;
			kinect.setCameraTiltAngle(angle);
			break;
			
		case OF_KEY_DOWN:
			angle--;
			if(angle<-30) angle=-30;
			kinect.setCameraTiltAngle(angle);
			break;
	}
}

//--------------------------------------------------------------
void audMode::AudmouseDragged(int x, int y, int button)
{}

//--------------------------------------------------------------
void audMode::AudmousePressed(int x, int y, int button)
{}

//--------------------------------------------------------------
void audMode::AudmouseReleased(int x, int y, int button)
{}

//--------------------------------------------------------------
void audMode::AudwindowResized(int w, int h)
{}
