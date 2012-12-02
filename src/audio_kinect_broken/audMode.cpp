#include "audMode.h"
#include "ofAppGlutWindow.h"

//v1


//--------------------------------------------------------------
audMode::audMode(){
	
}

audMode::~audMode(){
	
}


void audMode::setup() {
	ofSetLogLevel(OF_LOG_VERBOSE);
	
	c1.setHex(0xF9CDAD); //butternut squash
	c2.setHex(0xFFD700); //cornucopia
	c3.setHex(0xFC9D9A); //peony
	c4.setHex(0x83AF9B); //Duck's Egg
	//c5.setHex(0xC8C8A9); //Timothy Hay
	alpha = 255;
	
	/*------------------------------------------------------*/
	// enable depth->video image calibration
	kinect.setRegistration(true);
	kinect.init();
	//kinect.init(true); // shows infrared instead of RGB video image
	//kinect.init(false, false); // disable video image (faster fps)
	kinect.open();		// opens first available kinect
	/*------------------------------------------------------*/
	
	
	/*------------------------------------------------------*/
	vidGrabber.setDeviceID(5);
	vidGrabber.initGrabber( 320, 240 );
	/*------------------------------------------------------*/
	
	
	/*---------------allocate images----------------------------------*/
	colorImage.allocate( 320, 240 );
	grayImage.allocate(kinect.width, kinect.height);
	grayThreshNear.allocate(kinect.width, kinect.height);
	grayThreshFar.allocate(kinect.width, kinect.height);
	/*---------------------------------------------------------------*/
	
	
	/*---------------init arduino stuffs-----------------------------*/
	// this will print out all of the devices attached to your computer
	serial.enumerateDevices();
	//serial.setup("COM4"); // windows will look something like this.
	serial.setup("/dev/tty.usbmodem641",9600); // mac osx looks like this
	//serial.setup("/dev/ttyUSB0", 9600); //linux looks like this
	/*---------------------------------------------------------------*/
	
	printf(".................setting up................\n");
	serial.listDevices();
	
	nearThreshold = 255;
	farThreshold = 255;
	
	ofSetFrameRate(60);
	
	// zero the tilt on startup
	angle = 20;
	kinect.setCameraTiltAngle(angle);
	// start from the front
	bDrawPointCloud = false;
}

//--------------------------------------------------------------
void audMode::update() {
	
	ofBackground(100, 100, 100);
	kinect.update();
	
	/*-------------------arduino stuff--------------------*/
	int i, len, largest;
	
	vidGrabber.grabFrame();
	if (vidGrabber.isFrameNew()) {
		colorImage = vidGrabber.getPixels();
        grayImage = colorImage;

		// find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
		// also, find holes is set to true so we will get interior contours as well....
		contourFinder.findContours(grayImage, 10, (kinect.width*kinect.height)/2, 20, false);

		/*Now that the contours have been processed we can try to figure out which is the largest 
		 since that’s probably the one we’re interested in.*/
		largest = -1;
		printf("contourFinder.blobs.size() = %d\n", int(contourFinder.blobs.size()));
		if(contourFinder.blobs.size() > 0) {
			largest = 0;
			//why do we skip the last one? if doesn't work subtract 1 from condition portion of loop
			for (int i = 1; i < contourFinder.blobs.size(); i++) {
				if(contourFinder.blobs.at(i).area > contourFinder.blobs.at(largest).area) {
					largest = i;
				}
			}
			printf("largest = %d", largest);
			// this is important: we don't want to send data to the serial port
			// on every frame because the Arduino runs much more slowly than the
			// oF app.
			if(ofGetFrameNum() % 6 == 0) {
				if(largest != -1) {
					//serial.writeByte(contourFinder.blobs.at(largest).centroid.y/kinect.height * 255);
					//printf("<#message#>");
					//serial.writeByte(contourFinder.blobs.at(largest).centroid.x/kinect.width * 255);
				}
			}
		}
	}
	/*-----------------------------------------------------------------------------------*/
	
}

//--------------------------------------------------------------
void audMode::draw() {
	/*	ofEnableAlphaBlending();
	 ofBackground(50,1);
	 ofDisableAlphaBlending();
	 
	 easyCam.begin();
	 ofSetColor(0,255,255);
	 drawPointCloud();
	 easyCam.end();*/
	ofSetColor( 255, 255, 255);	
	printf("grayImage %s\n", kinect.getDepthPixels());
	kinect.drawDepth(10, 10, 400, 300);
	kinect.draw(420, 10, 400, 300);
	
	grayImage.draw(10, 320, 400, 300);
	contourFinder.draw(10, 320, 400, 300);
	
}

void audMode::drawPointCloud() {	
	int w = 640;
	int h = 480;
	ofMesh mesh;
	ofMesh mesh_r;
	mesh.setMode(OF_PRIMITIVE_POINTS);
	mesh_r.setMode(OF_PRIMITIVE_POINTS);
	int step = 3;
	for(int y = 0; y < h; y += step) {
		for(int x = 0; x < w; x += step) {
			if(kinect.getDistanceAt(x, y) > 0) {
				mesh_r.addColor(ofColor(255, 255, 255, alpha));
				mesh_r.addVertex(kinect.getWorldCoordinateAt(x, y));
				mesh.addColor(ofColor(230, 230, 230, alpha));
				mesh.addVertex(kinect.getWorldCoordinateAt(x, y));
				ofVec3f tmp = kinect.getWorldCoordinateAt(x, y);
				//printf("%d %d %d\n", tmp.x, tmp.y, tmp.z );
			}
		}
	}
	glPointSize(1);
	ofPushMatrix();
	// the projected points are 'upside down' and 'backwards' 
	ofScale(-1, -1, -1);
	ofTranslate(w/2, 0, -1000); // center the points a bit
	//ofTranslate(0, 0, -1000); // center the points a bit
	glEnable(GL_DEPTH_TEST);
	int counter = 0;
	mesh.drawVertices(); 
	//mesh.drawWireframe();
	glDisable(GL_DEPTH_TEST);
	ofPopMatrix();
	
	
	glPointSize(1);
	ofPushMatrix();
	// the projected points are 'upside down' and 'backwards' 
	ofScale(1, -1, -1);
	ofTranslate(-w/2, 0, -1000); // center the points a bit
	glEnable(GL_DEPTH_TEST);
	mesh_r.drawVertices();
	//mesh_r.drawFaces ();
	glDisable(GL_DEPTH_TEST);
	ofPopMatrix(); 
	
}

ofColor audMode::getColor(int x) {
	switch (x) {
		case 1:
			return c1;
			break;
		case 2:
			return c2;
			break;
		case 3:
			return c3;
			break;
		case 4:
			return c4;
			break;
		default:
			//return c5;
			break;
	}
}

//--------------------------------------------------------------
void audMode::exit() {
	kinect.setCameraTiltAngle(0); // zero the tilt on exit
	kinect.close();
	
}

//--------------------------------------------------------------
void audMode::AudkeyPressed (int key) {
	switch (key) {
		case ' ':
			bThreshWithOpenCV = !bThreshWithOpenCV;
			break;
			
		case'p':
			bDrawPointCloud = !bDrawPointCloud;			break;
			
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

