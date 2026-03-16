#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
	cam.setDistance(100);
	cam.setNearClip(.1);
	ofSetFrameRate(FRAME_RATE);

	background.load("pexels-pixabay-236970.jpg");

	light.setup();
	light.setPosition(0, BOUNDING_BOX_DEPTH, BOUNDING_BOX_HEIGHT);
	light.setDiffuseColor(ofColor(74, 71, 64));

	light2.setup();
	light2.setPosition(BOUNDING_BOX_WIDTH * 1.5, -BOUNDING_BOX_HEIGHT * 1.5, 0);
	light2.setDiffuseColor(ofColor(120, 114, 102));

	spotlight.setSpotlight();
	spotlight.setPosition(-BOUNDING_BOX_WIDTH, BOUNDING_BOX_DEPTH, 0);
	spotlight.lookAt(glm::vec3(0, 5, 0));
	spotlight.setSpotlightCutOff(45);
	spotlight.setSpotConcentration(0);
	spotlight.setDiffuseColor(ofColor::lightGoldenRodYellow);

	// gui panel setup
	gui.setup();
	gui.add(collision_avoidance_factor.setup("Collision Avoidance Factor", 50.0, 0.0, 100.0));
	gui.add(velocity_matching_factor.setup("Velocity Matching Factor", 27, 0.0, 50.0));
	gui.add(centering_factor.setup("Centering Factor", 8, 0.0, 40.0));
	gui.add(banking_factor.setup("Banking Factor", 1.0, 0.0, 1.0));
	gui.add(smoothing_factor.setup("Smoothing Factor", 0.5, 0.0, 1.0));
	gui.add(boundary_strength.setup("Boundary Strength", 30.0, 0.0, 100.0));
	gui.add(tmin.setup("T-min", ofVec3f(-17, -17, -17), ofVec3f(-50, -50, -50), ofVec3f(50, 50, 50)));
	gui.add(tmax.setup("T-max", ofVec3f(17, 17, 17), ofVec3f(-50, -50, -50), ofVec3f(50, 50, 50)));
	gui.add(tempoSlider.setup("Tempo", 0.5, 0.1, 5.0));
	gui.add(volume.setup("Volume", 1.0, 0.0, 1.0));
	gui.add(adjustPitch.setup("Pitch with velocity", false));
	bHide = false;
	drawBoundary = false;

	// set up background music loop
	bgm.load("560440__migfus20__calming-background-music-orchestra-loop.mp3");
	bgm.setLoop(true);
	bgm.setVolume(0.75 * volume);
	bgm.play();

	// create and add boids to a flock
	flock = new Flock();
	for (int i = 0; i < FLOCK_SIZE; i++) {
		Boid* b = new Boid();	// create new boid
		// assign boid with random initial position within bounding box
		b->position = ofVec3f(
			ofRandom(-BOUNDING_BOX_WIDTH * 0.9f, BOUNDING_BOX_WIDTH * 0.9f),
			ofRandom(-BOUNDING_BOX_HEIGHT * 0.9f, BOUNDING_BOX_HEIGHT * 0.9f),
			ofRandom(-BOUNDING_BOX_DEPTH * 0.9f, BOUNDING_BOX_DEPTH * 0.9f));
		// assign boid with initial baseline tempo based on i (to prevent overlapping tempos)
		b->baselineTempo = i + 5;
		b->tempo_i = i;
		flock->add(b);		// add boid to flock
	}
	flock->setupModel();	// load fbx boid models
	flock->setupSound();

	// add turbulence and boundary avoidance forces
	turbulenceForce = new TurbulenceForce(ofVec3f(tmin), ofVec3f(tmax));
	flock->addForce(turbulenceForce);
	boundaryForce = new BoundaryForce(5.0f, boundary_strength);
	flock->addForce(boundaryForce);
}

//--------------------------------------------------------------
void ofApp::update(){
	// update volume of bgm
	bgm.setVolume(0.75 * volume);
	// update acceleration scaler factors
	flock->updateScalerFactors(collision_avoidance_factor, velocity_matching_factor, centering_factor, banking_factor, smoothing_factor, tempoSlider, adjustPitch);
	// update turbulence force
	turbulenceForce->setTurbulence(ofVec3f(tmin), ofVec3f(tmax));

	// update + iterate flock
	flock->update();
	flock->playSound(volume); // TODO: move kalimba, piano, guitar setup to boid
}

//--------------------------------------------------------------
void ofApp::draw(){
	background.draw(0, 0, ofGetWidth(), ofGetHeight());

	cam.begin();
	
	// draw bounding box
	if (drawBoundary) {
		ofNoFill();
		ofDrawBox(0, 0, 0, 2 * BOUNDING_BOX_WIDTH, 2 * BOUNDING_BOX_HEIGHT, 2 * BOUNDING_BOX_DEPTH);
		ofFill();
	}

	ofEnableDepthTest();	// lighting ON
	ofEnableLighting();
	light.enable();
	light2.enable();
	spotlight.enable();

	// calculate billboard matrix before drawing flock
	// get camera's rotation matrix (upper left 3x3 matrix)
	ofMatrix4x4 camMatrix = glm::mat3(cam.getModelViewMatrix());
	// invert camera's rotation matrix to get billboard matrix
	ofMatrix4x4 billboardMatrix = camMatrix.getInverse();

	// draw flock
	flock->draw(billboardMatrix);

	light.disable();		// lighting OFF
	light2.disable();
	spotlight.disable();
	ofDisableLighting();
	ofDisableDepthTest();

	cam.end();

	if (!bHide) gui.draw();

	// draw screen data
	string str;
	str += "Frame Rate: " + std::to_string(ofGetFrameRate());
	ofSetColor(ofColor::white);
	ofDrawBitmapString(str, ofGetWindowWidth() - 170, 15);
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
	switch (key) {
	case 'b':
		drawBoundary = !drawBoundary;
		break;
	case 'f':
		ofToggleFullscreen();
		break;
	case 'h':
		bHide = !bHide;
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
