#pragma once

#include "Boid.h"

const int FLOCK_SIZE = 50;

//  Pure Virtual Function Class - must be subclassed to create new forces
class BoidForce {
public:
	virtual void updateForce(Boid*) = 0;
};

class Flock {
public:
	~Flock();
	vector<Boid*> flock;
	vector<BoidForce*> forces;

	void add(Boid*);
	void remove(int);
	void addForce(BoidForce*);
	void update();
	void draw(ofMatrix4x4);
	void updateNeighbors();
	void setupSound();
	void playSound(float);
	void updateScalerFactors(float, float, float, float, float, float, bool);

	void setupModel() {
		for (Boid* b : flock) {
			b->setupModel();
		}
	}

	void updateModel() {
		for (Boid* b : flock) {
			b->updateModel();
		}
	}
};

class TurbulenceForce : public BoidForce {
public:
	ofVec3f tmin, tmax;

	TurbulenceForce(const ofVec3f& min, const ofVec3f& max) {
		tmin = min;
		tmax = max;
	};
	void updateForce(Boid*);
	void setTurbulence(ofVec3f newTmin, ofVec3f newTmax) {
		tmin = newTmin;
		tmax = newTmax;
	};
};

class BoundaryForce : public BoidForce {
public:
	float threshold;
	float strength;

	BoundaryForce(const float thres, const float stre) {
		threshold = thres;
		strength = stre;
	};
	void updateForce(Boid*);
	void setBoundary(float thres, float stre) {
		threshold = thres;
		strength = stre;
	}
};

