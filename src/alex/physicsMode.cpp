#include "physicsMode.h"

physicsMode::physicsMode(){
	birthRate = 0;
	maxParticles = 800;
}

void physicsMode::setup(){
	srcImg.allocate(256, 256, OF_IMAGE_COLOR_ALPHA);
    srcImg.loadImage("source.png");

	sources.push_back(source(ofVec3f(ofGetWidth()/2+10, ofGetHeight()/2+10, 0), physicsMode::source::ORBIT, srcImg));
//	sources.push_back(source(ofVec3f(ofGetWidth()/2-10, ofGetHeight()/2-10, 0), physicsMode::source::SINK, srcImg));
//	sources.push_back(source(ofVec3f(ofGetWidth()/2, ofGetHeight()/2, 0), physicsMode::source::SINK, srcImg));
	//addParticles(800);
}

void physicsMode::update(){
	for(vector<source>::iterator e = sources.begin(); e != sources.end(); ++e){
		for(vector<physicsMode::source::particle>::iterator p = particles.begin(); p != particles.end(); ++p){
			p->applyForce(*e, e->mass*10);
		//	p->update();
		}
	}
	for(vector<physicsMode::source::particle>::iterator p = particles.begin(); p != particles.end();){
		if(p->isDead)
			p = particles.erase(p);
		else
			p++;
		
	}
	//printf("particle array size %d", particles.size());
}

void physicsMode::render(){
	for(vector<source>::iterator e = sources.begin(); e != sources.end(); e++){
		e->render();
	}
	printf("\n");
	for(vector<physicsMode::source::particle>::iterator p = particles.begin(); p != particles.end(); p++)
		p->render();
}
/*--------------------------------------------------*
Update Sources

update the source particles with relevant data 
from the main app
 *--------------------------------------------------*/
void physicsMode::updateSources(float vol, ofColor c, bool isChanged, bool isKick, bool isSnare){
	int distThresh = 51;

		repulseSources();
		for(vector<source>::iterator e1 = sources.begin(); e1 != sources.end(); ++e1){
			if(isChanged)
				e1->col = c;
			e1->radius = vol;
			e1->mass = vol*2;
			e1->pullToCenter(vol*2);
			e1->update(isKick, isSnare);
		}
}

void physicsMode::repulseSources(){
	for(vector<source>::iterator e1 = sources.begin(); e1 != sources.end(); ++e1){
		for(vector<source>::iterator e2 = sources.begin(); e2 != sources.end(); ++e2){
			e1->attract(*e2,300);
			ofVec3f dir = e1->loc - e2->loc;
            float distSqrd = dir.lengthSquared();
            
            if( distSqrd > 0.0f ){
                dir.normalize();
                float F = 1.0f/distSqrd;
                    
                e1->acc += dir * ( F / e1->mass );
                e2->acc -= dir * ( F / e2->mass );
			}
		}
	}
}

void physicsMode::mousePressed(physicsMode::source::Type t, ofVec3f pos){
	if(sources.size()<3)
		sources.push_back(source(pos, t, srcImg));
}
void physicsMode::addParticles(int amt){
	if(particles.size() < maxParticles){
	for(int i=0; i<amt; i++)
		particles.push_back(physicsMode::source::particle());}
}
/*--------------------------------*
	Source Class
 *--------------------------------*/
void physicsMode::source::render(){
	ofPushStyle();
	ofSetColor(col);
	float imgRad = radius*5 +10;
	float radMult = 0.08;
	float radSq = 50 + (radius*radius) * radMult;

	if(radSq > 200){
		ofCircle(loc.x, loc.y, (radSq * ((radSq/2)*.009))*radMult);
		ofSetColor(40);
		ofCircle(loc.x, loc.y, (radSq * ((radSq/2)*.005))*radMult);
		ofSetColor(col);
		ofCircle(loc.x, loc.y, radSq/2*radMult);
	}
	else{
		ofCircle(loc.x, loc.y, radSq/2*radMult);
	}
//	spark.draw(loc.x-imgRad/2,loc.y-imgRad/2,imgRad,imgRad);

	renderParticles();
	ofPopStyle();
}

physicsMode::source::source(ofVec3f initPos, Type _type, ofImage s){
	loc = initPos;
	vel = ofVec3f(0,0,0);
	acc = ofVec3f(0,0,0);
	radius = 100;
	mass = 50;
	spark = s;
	type = _type;
	col = ofColor(255,0,255);
	charge = 10; //set to music

	addParticles(20);
}

void physicsMode::source::update(bool isKick, bool isSnare){
	vel = vel+acc;

	vel.limit(5); //BPM
	loc = loc + vel;
	vel = vel*.98;
	//acc = acc * .1;
	//vel = ofVec3f(0,0,0);
	acc = ofVec3f(0,0,0);

	updateParticles(isKick, isSnare);
}

void physicsMode::source::attract(source s, float range){
	float maxDist = ofGetWidth()/2;
	ofVec3f m = ofVec3f(0,0,0);
    ofVec3f dirToPull = ofVec3f(loc.x, loc.y, 0);
    dirToPull = dirToPull - s.loc;
    float distToPull = dirToPull.length();

	if(distToPull > 1){
			//straight up attraction/repuslion forces
			float theta, F;
			F = mass * s.mass;
			m.x = (mass*loc.x + s.mass*s.loc.x)/(mass+s.mass);
			m.y = (mass*loc.y + s.mass*s.loc.y)/(mass+s.mass);

			if(distToPull < range){
				theta = findAngle( loc.x - m.x, loc.y - m.y);
			//if(distToPull > range)
			//	 theta = findAngle( m.x - loc.x, m.y - loc.y );
    
			m.x = (F*cos(theta)) / distToPull;
			m.y = (F*sin(theta)) / distToPull;
    
			theta = physicsMode::source::findAngle(m.x, m.y);
			acc.x += (m.length() * cos(theta));
			acc.y += (m.length() * sin(theta)); 
			}
	}
}

void physicsMode::source::pullToCenter(float distThresh)
  {
	ofVec3f center = ofVec3f(ofGetWidth()/2, ofGetHeight()/2);
    ofVec3f dirToCenter = ofVec3f(loc.x, loc.y, loc.z);
    dirToCenter= dirToCenter - center;
	float distToCenter = dirToCenter.length();

    
    if(distToCenter >distThresh){
      dirToCenter.normalize();
      float pullStrength = 0.001f;
      dirToCenter= dirToCenter*(((distToCenter -distThresh))*pullStrength);
      vel= vel - (dirToCenter);
    }
  }

float physicsMode::source::findAngle(float x1, float y1, float x2, float y2){
  float xd = x1 - x2;
  float yd = y1 - y2;

  float t = atan2(yd,xd);
  float a = (180 + (-(180 * t) / PI));
  return a;
}
float physicsMode::source::findAngle(float x, float y){
  float theta;
  if(x == 0) {
    if(y > 0) 
      theta = PI/2.0;
    else if(y < 0) 
      theta = PI/2.0;
    else 
      theta = 0;
  }
  else {
    theta = atan( y / x );
    if(( x < 0 ) && ( y >= 0 )) { theta += PI; }
    if(( x < 0 ) && ( y < 0 )) { theta -= PI; }
  }
  return theta;
}
/*---------PARTICLE CONTROL-----------*/
void physicsMode::source::updateParticles(bool isKick, bool isSnare){
	repulseParticles();
	for(vector<particle>::iterator e1 = mParticles.begin(); e1 != mParticles.end(); ++e1){
		float radMult = 0.09;
		float radSq = (radius*radius) * radMult;

		//volume is stored in the radius
		//e1->pull(*this,radius*3+radius); //outer ring limit
		e1->pull(*this,(radSq * ((radSq/2)*.009))*radMult);
		e1->push(*this,radius*3+10);  //inner ring limit

		if(isKick){
			e1->orbit(*this, radius*5);
		}
		e1->update(isSnare);
	}
}
void physicsMode::source::renderParticles(){
	for(vector<particle>::iterator e1 = mParticles.begin(); e1 != mParticles.end(); ++e1){
		e1->render();
	}
}
void physicsMode::source::repulseParticles(){
	for(vector<particle>::iterator e1 = mParticles.begin(); e1 != mParticles.end(); ++e1){
		for(vector<particle>::iterator e2 = e1; e2 != mParticles.end(); ++e2){

			ofVec3f dir = e1->loc - e2->loc;
            float distSqrd = dir.lengthSquared();
            
            if( distSqrd > 0.0f ){
                dir.normalize();
                float F = 1.0f/distSqrd;
                    
                e1->acc += dir * ( F / e1->mass );
                e2->acc -= dir * ( F / e2->mass );
			}
		}
	}
}

vector<physicsMode::source::particle> physicsMode::source::addParticles(int num){
	for(int i=0; i<num; i++){
		//parametric equations for a circle
		float u = ofRandom(2*PI);
		float v = ofRandom(-PI/2, PI/2);
		particle p = particle(ofVec3f(loc.x + (radius + 10)*cos(v)*sin(u),loc.y + (radius + 10)*cos(u)*cos(v), loc.z + (radius+10)*sin(v)), 
							  ofRandom(2,5), 100);
		mParticles.push_back(p);
	}
	return mParticles;
}

/*--------------------------------*
	Particle Class
 *--------------------------------*/
physicsMode::source::particle::particle(){
	loc = ofVec3f(ofRandom(0,ofGetWidth()),ofRandom(0,ofGetHeight()),0);
	mass = ofRandom(1,5);
	maxSpeed = 5;
	magnitude = 0;
	angle = 0;
	death = 0.9;
	age = 0;
	lifespan = 600;
	isDead = false;
	vel = ofVec3f(ofRandom(-3,3),ofRandom(-3,3),0);
	acc = ofVec3f(ofRandom(-3,3),ofRandom(-3,3),0);
}
physicsMode::source::particle::~particle(){
}
physicsMode::source::particle::particle(ofVec3f _loc, float m, int life){
	loc = _loc;
	mass = m;
	maxSpeed = 9;
	magnitude = 0;
	angle = 0;
	death = 0.99;
	age = 0;
	lifespan = life;
	isDead = false;
	col = colorGen.getColor(50, colorGen.getColourConstraints(CT_FRESH));
	analgCol = col;
	//vel = ofVec3f(ofRandom(-15,15),ofRandom(-15,15),0);
	//acc = ofVec3f(ofRandom(-15,15),ofRandom(-15,15),0);
}

void physicsMode::source::particle::render(){
	ofPushStyle();
	ofFill();
	ofSetColor(analgCol);
	ofCircle(loc.x, loc.y, 5);
	ofLine(pLoc, loc);
	ofPopStyle();
}

void physicsMode::source::particle::update(bool isSnare){

	pLoc = loc;
	vel = vel+acc;
	vel.limit(maxSpeed);
	loc = loc + vel;
	acc = ofVec3f(0,0,0);
	//vel = vel*death;
	age++;
	if(age > lifespan)
		isDead = true;

	if(isSnare){
		analgCol = colorGen.createRangeFromAnalogous(col)[(int)ofRandom(0,4)];
	}
}

void physicsMode::source::particle::pull(source s, float range){

	ofVec3f m = ofVec3f(0,0,0);
    ofVec3f dirToPull = ofVec3f(loc.x, loc.y, 0);
    dirToPull = dirToPull - s.loc;
    float distToPull = dirToPull.length();

	if(distToPull > 1){
			//straight up attraction/repuslion forces
			float theta, F;
			F = mass * s.mass;
			m.x = (mass*loc.x + s.mass*s.loc.x)/(mass+s.mass);
			m.y = (mass*loc.y + s.mass*s.loc.y)/(mass+s.mass);

			if(distToPull > range){
			 theta = findAngle( m.x - loc.x, m.y - loc.y );
			m.x = (F*cos(theta)) / distToPull;
			m.y = (F*sin(theta)) / distToPull;
    
			theta = findAngle(m.x, m.y);
			acc.x += (m.length() * cos(theta));
			acc.y += (m.length() * sin(theta)); 
			}
	}
}
void physicsMode::source::particle::push(source s, float range){
	ofVec3f m = ofVec3f(0,0,0);
    ofVec3f dirToPull = ofVec3f(loc.x, loc.y, 0);
    dirToPull = dirToPull - s.loc;
    float distToPull = dirToPull.length();

	if(distToPull > 1){
			//straight up attraction/repuslion forces
			float theta, F;
			F = mass * s.mass;
			m.x = (mass*loc.x + s.mass*s.loc.x)/(mass+s.mass);
			m.y = (mass*loc.y + s.mass*s.loc.y)/(mass+s.mass);

			if(distToPull < range){
			 theta = findAngle( loc.x - m.x, loc.y - m.y);
    
			m.x = (F*cos(theta)) / distToPull;
			m.y = (F*sin(theta)) / distToPull;
    
			theta = findAngle(m.x, m.y);
			acc.x += (m.length() * cos(theta));
			acc.y += (m.length() * sin(theta)); 
			}
	}
}
void physicsMode::source::particle::push(particle s, float range){
		ofVec3f m = ofVec3f(0,0,0);
    ofVec3f dirToPull = ofVec3f(loc.x, loc.y, 0);
    dirToPull = dirToPull - s.loc;
    float distToPull = dirToPull.length();

	if(distToPull > 1){
			//straight up attraction/repuslion forces
			float theta, F;
			F = mass * s.mass;
			m.x = (mass*loc.x + s.mass*s.loc.x)/(mass+s.mass);
			m.y = (mass*loc.y + s.mass*s.loc.y)/(mass+s.mass);

			if(distToPull < range){
			theta = findAngle( loc.x - m.x, loc.y - m.y);
    
			m.x = (F*cos(theta)) / distToPull;
			m.y = (F*sin(theta)) / distToPull;
    
			theta = findAngle(m.x, m.y);
			acc.x += (m.length() * cos(theta));
			acc.y += (m.length() * sin(theta)); 
			}
	}
}

void physicsMode::source::particle::orbit(source s, float range){
	ofVec3f m = ofVec3f(0,0,0);
    ofVec3f dirToPull = ofVec3f(loc.x, loc.y, 0);
    dirToPull = dirToPull - s.loc;
    float distToPull = dirToPull.length();

	if(distToPull < range){
		float deathThresh = (range - distToPull) / range;
		deathThresh = deathThresh * deathThresh;
		if(deathThresh > 0.95){
		//	isDead = true; //possibly respawn?
		}

		float F = mass * s.mass;
		dirToPull = dirToPull.normalize();
		ofVec3f tanForce = ofVec3f(dirToPull.y, - dirToPull.x, 0);
		tanForce = tanForce * (deathThresh*10);
		acc = (acc + tanForce);
	}
}

void physicsMode::source::particle::applyForce(source a, float range){
	ofVec3f m = ofVec3f(0,0,0);
    ofVec3f dirToPull = ofVec3f(loc.x, loc.y, 0);
    dirToPull = dirToPull - a.loc;
    float distToPull = dirToPull.length();

	if(distToPull < range){
		float deathThresh = (range - distToPull) / range;
		deathThresh = deathThresh * deathThresh;
		if(deathThresh > 0.95){
			isDead = true; //possibly respawn?
		}
		if((a.type == physicsMode::source::EMIT) || a.type == physicsMode::source::SINK){
			//straight up attraction/repuslion forces
			float theta, F;
			F = mass * a.mass;
			m.x = (mass*loc.x + a.mass*a.loc.x)/(mass+a.mass);
			m.y = (mass*loc.y + a.mass*a.loc.y)/(mass+a.mass);

			if(a.type == physicsMode::source::EMIT)
				theta = findAngle( loc.x - m.x, loc.y -m.y);
			else if(a.type == physicsMode::source::SINK)
				 theta = findAngle( m.x - loc.x, m.y - loc.y );
    
			m.x = (F*cos(theta)) / distToPull;
			m.y = (F*sin(theta)) / distToPull;
    
			angle = findAngle(m.x, m.y);
			acc.x += (m.length() * cos(angle));
			acc.y += (m.length() * sin(angle)); 
		}
		else{
			//orbit
			float F = mass * a.mass;
			dirToPull = dirToPull.normalize();
			ofVec3f tanForce = ofVec3f(dirToPull.y, - dirToPull.x, 0);
			tanForce = tanForce * (deathThresh*10);
			acc = (acc + tanForce);
		}
	}
}


float physicsMode::source::particle::findAngle(float x, float y){
  float theta;
  if(x == 0) {
    if(y > 0) {
      theta = HALF_PI;
    }
    else if(y < 0) {
      theta = 3*HALF_PI;
    }
    else {
      theta = 0;
    }
  }
  else {
    theta = atan( y / x );
    if(( x < 0 ) && ( y >= 0 )) { theta += PI; }
    if(( x < 0 ) && ( y < 0 )) { theta -= PI; }
  }
  return theta;
}


