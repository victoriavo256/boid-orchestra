#include "Flock.h"

Flock::~Flock() {
	// deallocate flock
	for (Boid* b : flock) delete b;
}

void Flock::add(Boid* b) {
	flock.push_back(b);
}

void Flock::remove(int i) {
	delete flock[i];
	flock.erase(flock.begin() + i);
}

void Flock::addForce(BoidForce* f) {
	forces.push_back(f);
}

void Flock::playSound(float vol) {
	// update volume and plays sound
	for (Boid* b : flock) {
		b->volume = vol;
		b->playSound();
	}
}

void Flock::updateNeighbors() {
	// update neighbor list for all birds in the flock
	for (Boid* b : flock) {
		b->neighbors.clear();			// reset neighbors list
		b->tempo = b->baselineTempo;	// reset tempo to baseline

		for (Boid* c : flock) {
			if (b == c) continue; // skip self

			// calculate distance that other boid is from this boid
			float dist = glm::distance(glm::vec3(b->position), glm::vec3(c->position));

			// if boid is within given proximity, add to neighbors list
			if (dist < MAX_DIST_THRESHOLD) {
				b->neighbors.push_back(c);
			}
		}

		// adjust tempo depending on how many neighbors there are, with a clamp of 7
		if (b->neighbors.size() > 10) {
			b->tempo /= 10;
		}
		else {
			b->tempo /= b->neighbors.size();
		}
	}
}

void Flock::update() {
	// if flock is empty, just return
	if (flock.empty()) return;

	// update neighbors list on all boids
	updateNeighbors();

	// update forces on all boids
	for (Boid* b : flock) {
		for (BoidForce* f : forces) {
			f->updateForce(b);
		}
	}

	// update frame of wing flap animation
	updateModel();

	// integrate all the boids in flock
	for (Boid* b : flock) {
		b->integrate();
	}

}

void Flock::draw(ofMatrix4x4 billboardMatrix) {
	for (Boid* b : flock) {
		b->draw(billboardMatrix);
	}
}

void Flock::updateScalerFactors(float ca, float vm, float cf, float bf, float sf, float ts, bool ap) {
	for (Boid* b : flock) {
		b->collision_avoidance_factor = ca;
		b->velocity_matching_factor = vm;
		b->centering_factor = cf;
		b->banking_factor = bf;
		b->smoothing_factor = sf;
		b->baselineTempo = (b->tempo_i / ts) + 5;
		b->adjustPitch = ap;
	}
}

void Flock::setupSound() {
	for (Boid* b : flock) {
		b->setupSound();
	}
}

void TurbulenceForce::updateForce(Boid* b) {
	// add a little "noise" to the boid's forces to achieve a more natural look to the motion
	float t = ofGetElapsedTimef(); // time-based variation
	b->forces.x += ofMap(ofNoise(b->position.x * 0.1, t), 0.45, 0.55, tmin.x, tmax.x);
	b->forces.y += ofMap(ofNoise(b->position.y * 0.1, t + 100), 0.45, 0.55, tmin.y, tmax.y);
	b->forces.z += ofMap(ofNoise(b->position.z * 0.1, t + 200), 0.45, 0.55, tmin.z, tmax.z);
}

void BoundaryForce::updateForce(Boid* b) {
	// add a boundary avoidance force as boids get closer to bounding walls

	// x walls
	if (b->position.x < -BOUNDING_BOX_WIDTH + threshold) 
		b->forces.x += strength * (1.0f - (b->position.x - -BOUNDING_BOX_WIDTH) / threshold);
	else if (b->position.x > BOUNDING_BOX_WIDTH - threshold)
		b->forces.x -= strength * (1.0f - (BOUNDING_BOX_WIDTH - b->position.x) / threshold);

	// y walls
	if (b->position.y < -BOUNDING_BOX_HEIGHT + threshold)
		b->forces.y += strength * (1.0f - (b->position.y - -BOUNDING_BOX_HEIGHT) / threshold);
	else if (b->position.y > BOUNDING_BOX_HEIGHT - threshold)
		b->forces.y -= strength * (1.0f - (BOUNDING_BOX_HEIGHT - b->position.y) / threshold);

	// z walls
	if (b->position.z < -BOUNDING_BOX_DEPTH + threshold)
		b->forces.z += strength * (1.0f - (b->position.z - -BOUNDING_BOX_DEPTH) / threshold);
	else if (b->position.z > BOUNDING_BOX_DEPTH - threshold)
		b->forces.z -= strength * (1.0f - (BOUNDING_BOX_DEPTH - b->position.z) / threshold);
}

