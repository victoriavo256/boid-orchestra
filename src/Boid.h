#include "ofMain.h"
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
#include "ofxAssimpModelLoader.h"

const float FRAME_RATE = 60.0f;
const float BOUNDING_BOX_SIZE = 30.0f;
const float BOUNDING_BOX_HEIGHT = 40.0f;
const float BOUNDING_BOX_WIDTH = 60.0f;
const float BOUNDING_BOX_DEPTH = 60.0f;
const float MIN_DIST_THRESHOLD = 10.0f;
const float MAX_DIST_THRESHOLD = 15.0f;
const float MIN_THETA_THRESHOLD = 90.0f;
const float MAX_THETA_THRESHOLD = 270.0f;

class Boid {
public:
	Boid();
	~Boid();
	ofVec3f	position;
	ofVec3f	velocity;
	ofVec3f	acceleration;
	ofVec3f	forces;
	float	damping;
	float   mass;
	float	radius;
	float	banking;
	float	collision_avoidance_factor;
	float	velocity_matching_factor;
	float	centering_factor;
	float	banking_factor;
	float	smoothing_factor;
	vector<Boid*> neighbors;
	ofxAssimpModelLoader model;

	bool	soundPlayed;
	float	soundStartTime = 0.0f;
	float	pitch;
	bool	adjustPitch;	// if true, adjust pitch with the boid's velocity
	float	volume;
	float	tempo;
	float	baselineTempo;
	float	tempo_i;
	float	timeOfLastSound;

	vector<ofxAssimpModelLoader*> frames;
	int currentFrame = 0;
	float secondsPerFrame = 0.1f;
	float secondsSinceLastFrame = 0.0f;

	// kalimba sounds
	ofSoundPlayer	kalimba_db1;
	ofSoundPlayer	kalimba_f4;
	ofSoundPlayer	kalimba_c1;
	ofSoundPlayer	kalimba_c2;
	ofSoundPlayer	kalimba_c3;
	std::vector<ofSoundPlayer> kalimba;
	/* ^ kalimba sound effects credits:
	* Db1.aif by UbikPhonik -- https://freesound.org/s/177950/ -- License: Attribution 4.0
	* KLMB f#4.wav by kostasvomvolos -- https://freesound.org/s/6999/ -- License: Attribution NonCommercial 4.0
	* Kalimba C1 by dvdfu -- https://freesound.org/s/536551/ -- License: Creative Commons 0
	* Kalimba C2 by dvdfu -- https://freesound.org/s/536550/ -- License: Creative Commons 0
	* Kalimba C3 by dvdfu -- https://freesound.org/s/536549/ -- License: Creative Commons 0
	*/

	// piano sounds
	ofSoundPlayer doh;
	ofSoundPlayer re;
	ofSoundPlayer mi;
	ofSoundPlayer fa;
	ofSoundPlayer sol;
	ofSoundPlayer la;
	ofSoundPlayer si;
	std::vector<ofSoundPlayer> piano;
	// ^piano sound effects credits:
	// Sound Effects by freesound_community from Pixabay

	// guitar sounds
	ofSoundPlayer guitar_a19;
	ofSoundPlayer guitar_d15;
	ofSoundPlayer guitar_g11;
	ofSoundPlayer guitar_b13;
	ofSoundPlayer guitar_g12;
	ofSoundPlayer guitar_d16;
	ofSoundPlayer guitar_a20;
	std::vector<ofSoundPlayer> guitar;
	/* ^guitar sound effects credits:
	* guitar tones 001 string A 19 tone E4.wav by josefpres -- https://freesound.org/s/681853/ -- License: Creative Commons 0
	* guitar tones 001 string D 15 tone F4.wav by josefpres -- https://freesound.org/s/681888/ -- License: Creative Commons 0
	* guitar tones 001 string G 11 tone FIS4.wav by josefpres -- https://freesound.org/s/681908/ -- License: Creative Commons 0
	* guitar tones 001 string B 13 tone C5.wav by josefpres -- https://freesound.org/s/681934/ -- License: Creative Commons 0
	* guitar tones 001 string G 12 tone G4.wav by josefpres -- https://freesound.org/s/681909/ -- License: Creative Commons 0
	* guitar tones 001 string D 16 tone FIS4.wav by josefpres -- https://freesound.org/s/681889/ -- License: Creative Commons 0
	* guitar tones 001 string A 20 tone F4.wav by josefpres -- https://freesound.org/s/681854/ -- License: Creative Commons 0
	*/

	void    draw(ofMatrix4x4);
	void    integrate();
	void	computeFlockingForces(glm::vec3&, glm::vec3&, glm::vec3&);
	void	computeBanking();
	void	playSound();

	// returns a transform matrix that rotates v1 onto v2
	glm::mat4 rotateToVector(glm::vec3 v1, glm::vec3 v2) {
		glm::vec3 axis = glm::cross(v1, v2);
		glm::quat q = glm::angleAxis(glm::angle(v1, v2), glm::normalize(axis));
		return glm::toMat4(q);
	}

	// loads all 8 frames of the 3d boid wing flap animation
	void setupModel() {
		for (int i = 1; i <= 8; ++i) {
			ofxAssimpModelLoader* m = new ofxAssimpModelLoader();
			m->loadModel("boid_" + ofToString(i) + ".fbx");
			m->setScale(0.01f, 0.01f, 0.01f);
			frames.push_back(m);
		}
	}

	// plays wing flap animation frame by frame
	void updateModel() {
		if (ofGetElapsedTimef() - secondsSinceLastFrame >= secondsPerFrame) {
			currentFrame = (currentFrame + 1) % frames.size();
			secondsSinceLastFrame = ofGetElapsedTimef();
		}
	}

	void setupSound() {
		// set up kalimba sound effects
		kalimba_db1.load("177950__ubikphonik__db1.mp3");
		kalimba_f4.load("6999__kostasvomvolos__klmb-f4.mp3");
		kalimba_c1.load("536551__dvdfu__kalimba-c1.mp3");
		kalimba_c2.load("536550__dvdfu__kalimba-c2.mp3");
		kalimba_c3.load("536549__dvdfu__kalimba-c3.mp3");
		kalimba.push_back(kalimba_db1);
		kalimba.push_back(kalimba_f4);
		kalimba.push_back(kalimba_c1);
		kalimba.push_back(kalimba_c2);
		kalimba.push_back(kalimba_c3);

		// set up piano sound effects
		doh.load("do.mp3");
		re.load("re.mp3");
		mi.load("mi.mp3");
		fa.load("fa.mp3");
		sol.load("sol.mp3");
		la.load("la.mp3");
		si.load("si.mp3");
		piano.push_back(doh);
		piano.push_back(re);
		piano.push_back(mi);
		piano.push_back(fa);
		piano.push_back(sol);
		piano.push_back(la);
		piano.push_back(si);

		// set up guitar sound effects
		guitar_a19.load("681853__josefpres__guitar-tones-001-string-a-19-tone-e4.wav");
		guitar_d15.load("681888__josefpres__guitar-tones-001-string-d-15-tone-f4.wav");
		guitar_g11.load("681908__josefpres__guitar-tones-001-string-g-11-tone-fis4.wav");
		guitar_b13.load("681934__josefpres__guitar-tones-001-string-b-13-tone-c5.wav");
		guitar_g12.load("681909__josefpres__guitar-tones-001-string-g-12-tone-g4.wav");
		guitar_d16.load("681889__josefpres__guitar-tones-001-string-d-16-tone-fis4.wav");
		guitar_a20.load("681854__josefpres__guitar-tones-001-string-a-20-tone-f4.wav");
		guitar.push_back(guitar_a19);
		guitar.push_back(guitar_d15);
		guitar.push_back(guitar_g11);
		guitar.push_back(guitar_b13);
		guitar.push_back(guitar_g12);
		guitar.push_back(guitar_d16);
		guitar.push_back(guitar_a20);
	}

};