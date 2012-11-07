#include "physicsMode.h"

physicsMode::physicsMode(){
	birthRate = 0;
	maxParticles = 800;
}

void physicsMode::mousePressed(source::Type t, ofVec3f pos){
	sources.push_back(source(pos, t));
}

void physicsMode::setup(){
	sources.push_back(source(ofVec3f(ofGetWidth()/2+10, ofGetHeight()/2+10, 0), source::ORBIT));
	sources.push_back(source(ofVec3f(ofGetWidth()/2-10, ofGetHeight()/2-10, 0), source::SINK));
	addParticles(800);
}

void physicsMode::addParticles(int amt){
	if(particles.size() < maxParticles){
	for(int i=0; i<amt; i++)
		particles.push_back(particle());}
}

void physicsMode::update(){
	for(vector<source>::iterator e = sources.begin(); e != sources.end(); ++e){
		for(vector<particle>::iterator p = particles.begin(); p != particles.end(); ++p){
			p->applyForce(*e, e->mass*10);
			p->update();
		}
	}
	for(vector<particle>::iterator p = particles.begin(); p != particles.end();){
		if(p->isDead)
			p = particles.erase(p);
		else
			p++;
		
	}
	printf("particle array size %d", particles.size());
}

void physicsMode::render(){
	for(vector<source>::iterator e = sources.begin(); e != sources.end(); e++){
		e->render();
	}
	printf("\n");
	for(vector<particle>::iterator p = particles.begin(); p != particles.end(); p++)
		p->render();
}

void physicsMode::updateSources(float vol){
	int distThresh = 51;

		repulseSources();
		for(vector<source>::iterator e1 = sources.begin(); e1 != sources.end(); ++e1){
			e1->radius = vol;
			e1->mass = vol;
			/*for(vector<source>::iterator e2 = sources.begin(); e2 != sources.end(); ++e2){
				e1->attract(*e2, 100);
			}*/
			e1->pullToCenter(vol*2);
			e1->update();
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

/*--------------------------------*
	Source Class
 *--------------------------------*/

physicsMode::source::source(ofVec3f initPos, Type _type){
	loc = initPos;
	//vel = ofVec3f(ofRandom(-1.0, 1.0),ofRandom(-1.0, 1.0),0);
	vel = ofVec3f(0,0,0);
	acc = ofVec3f(0,0,0);
	radius = 15;
	mass = 50;
	//acc = 0;

	type = _type;

	charge = 10; //set to music
}

void physicsMode::source::update(){
	vel = vel+acc;

	/*float gravity = 0.1;
	float gTheta = -(PI/180.0)*findAngle(loc.x,loc.y,ofGetWidth()/2, ofGetHeight()/2);
	ofVec2f gVel = ofVec2f(cos(gTheta), sin(gTheta));
	vel += gVel;*/

	vel.limit(5); //BPM
	loc = loc + vel;
	vel = vel*.98;
	//acc = acc * .1;
	//vel = ofVec3f(0,0,0);
	acc = ofVec3f(0,0,0);
}

void physicsMode::source::render(){
	ofPushStyle();
	if(type == EMIT){
		ofSetColor(0,255,255);
	}
	else{
		ofSetColor(255,0,255);
	}
	ofCircle(loc.x, loc.y, radius);
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
    
			theta = source::findAngle(m.x, m.y);
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
/*--------------------------------*
	Particle Class
 *--------------------------------*/
physicsMode::particle::particle(){
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
physicsMode::particle::~particle(){
}
physicsMode::particle::particle(ofVec3f _loc, float m, int life){
	loc = _loc;
	mass = m;
	maxSpeed = 4;
	magnitude = 0;
	angle = 0;
	death = 0.9;
	age = 0;
	lifespan = life;
	isDead = false;

	vel = ofVec3f(ofRandom(-15,15),ofRandom(-15,15),0);
	acc = ofVec3f(ofRandom(-15,15),ofRandom(-15,15),0);
}


void physicsMode::particle::update(){
	pLoc = loc;
	vel = vel+acc;
	vel.limit(maxSpeed);
	loc = loc + vel;
	acc = acc * death;
	age++;
	if(age > lifespan)
		isDead = true;
}


void physicsMode::particle::applyForce(source a, float range){
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
		if((a.type == source::EMIT) || a.type == source::SINK){
			//straight up attraction/repuslion forces
			float theta, F;
			F = mass * a.mass;
			m.x = (mass*loc.x + a.mass*a.loc.x)/(mass+a.mass);
			m.y = (mass*loc.y + a.mass*a.loc.y)/(mass+a.mass);

			if(a.type == source::EMIT)
				theta = findAngle( loc.x - m.x, loc.y -m.y);
			else if(a.type == source::SINK)
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


float physicsMode::particle::findAngle(float x, float y){
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

void physicsMode::particle::render(){
	ofPushStyle();
	ofSetColor(255,0,255);
	ofEllipse(loc.x, loc.y, 2,2);
	ofLine(pLoc, loc);
	ofPopStyle();
}
