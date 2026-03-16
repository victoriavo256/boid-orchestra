#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include "Flock.h"

class ofApp : public ofBaseApp {

public:
	void setup();
	void update();
	void draw();

	void keyPressed(int key);
	void keyReleased(int key);
	void mouseMoved(int x, int y);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void mouseEntered(int x, int y);
	void mouseExited(int x, int y);
	void windowResized(int w, int h);
	void dragEvent(ofDragInfo dragInfo);
	void gotMessage(ofMessage msg);

	ofEasyCam   cam;
	ofImage		background;
	ofLight		light;
	ofLight		light2;
	ofLight		spotlight;
	Flock*		flock;
	TurbulenceForce* turbulenceForce;
	BoundaryForce*	 boundaryForce;

	ofxPanel		gui;
	ofxFloatSlider	collision_avoidance_factor;
	ofxFloatSlider	velocity_matching_factor;
	ofxFloatSlider	centering_factor;
	ofxFloatSlider	banking_factor;
	ofxFloatSlider	smoothing_factor;
	ofxFloatSlider	boundary_strength;
	ofxVec3Slider	tmin;
	ofxVec3Slider	tmax;
	ofxFloatSlider	tempoSlider;
	ofxFloatSlider	volume = 1.0f;
	ofxToggle		adjustPitch;
	bool	bHide;
	bool	drawBoundary;

	float	tempo = 1 * (60.0f / 120);  // Seconds between triggers 
										// 1 = sound played per _ beats
										// 120 = bpm
	float	lastTriggerTime = 0.0f;
	ofSoundPlayer	bgm;
	// Calming Background Music Orchestra Loop by Migfus20 -- https://freesound.org/s/560440/ -- License: Attribution 4.0

};