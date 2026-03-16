#include "Boid.h"

Boid::Boid() {
	// initialize boid
	position.set(0, 0, 0);
	velocity.set(0, 0, 0);
	acceleration.set(0, 0, 0);
	forces.set(0, 0, 0);
	damping = .99;
	mass = 1;
	radius = 2;
	banking = 0;
	collision_avoidance_factor = 1.0f;
	velocity_matching_factor = 0.1f;
	centering_factor = 0.1f;
	banking_factor = 0.5f;
	smoothing_factor = 0.0f;
	soundPlayed = false;
	soundStartTime = 0.0f;
	pitch = 1.0f;
	volume = 1.0f;
	tempo = 0.0f;
	baselineTempo = 0.0f;
	tempo_i = 1.0f;
	timeOfLastSound = 0.0f;
}

Boid::~Boid() {
	// deallocate array of animation frames
	for (ofxAssimpModelLoader* f : frames) delete f;
	frames.clear();
}

void Boid::playSound() {
	float timeNow = ofGetElapsedTimef();
	if (timeNow - timeOfLastSound >= tempo) {
		// compute speed and map to pitch
		float speed = velocity.length();
		pitch = ofMap(speed, 0, 50, 0.75, 1.25, true);

		// play sound if tempo timing is right
		if (position.y > (BOUNDING_BOX_HEIGHT/3)) {
			int index = ofRandom(0, kalimba.size());
			if (adjustPitch) kalimba[index].setSpeed(pitch);
			kalimba[index].setVolume(volume);
			kalimba[index].play();
		}
		else if (position.y > -(BOUNDING_BOX_HEIGHT / 3)) {
			int index = ofRandom(0, piano.size());
			if (adjustPitch) piano[index].setSpeed(pitch);
			piano[index].setVolume(volume);
			piano[index].play();
		}
		else {
			int index = ofRandom(0, guitar.size());
			if (adjustPitch) guitar[index].setSpeed(pitch);
			guitar[index].setVolume(volume);
			guitar[index].play();
		}

		// update time of last sound
		timeOfLastSound = timeNow;

		// set flag to draw a circle when sound is played
		soundPlayed = true;
		soundStartTime = 0;
	}
}

void Boid::computeBanking() {
	// compute coordinate frame (u_x, u_y, u_z)
	glm::vec3 u_x = glm::normalize(glm::vec3(this->velocity));
	glm::vec3 u_y = glm::cross(glm::vec3(this->velocity), glm::vec3(this->acceleration)) / 
					glm::length(glm::cross(glm::vec3(this->velocity), glm::vec3(this->acceleration)));
	glm::vec3 cross = glm::cross(glm::vec3(this->velocity), glm::vec3(this->acceleration));

	// do this failsafe to prevent a NaN calculation
	if (glm::length(cross) < 1e-6) u_y = glm::vec3(0, 1, 0); // default up direction
	else u_y = glm::normalize(cross);

	glm::vec3 u_z = glm::normalize(glm::cross(u_x, u_y));

	// rotation matrix to align boid with its coordinate frame
	glm::mat4 rotMatrix = glm::mat4(
		glm::vec4(u_x, 0.0f),
		glm::vec4(u_y, 0.0f),
		glm::vec4(u_z, 0.0f),
		glm::vec4(0.0f, 0.0f, 0.0f, 1.0f)
	);

	// a_v is parallel to the boid's current velocity vector --> used to change the boid's speed
	glm::vec3 a_v = glm::dot(glm::vec3(this->acceleration), u_x) * u_x;

	// a_T is orthogonal to boid's current velocity vector --> used to induce turning
	glm::vec3 a_T = glm::vec3(this->acceleration) - a_v;

	// get banking angle
	float bankingAngle = glm::dot(glm::atan(banking_factor * a_T), u_z);

	// compute running average banking angle
	float avgBankingAngle = (((1 - smoothing_factor) * this->banking) + (smoothing_factor * bankingAngle));
	this->banking = avgBankingAngle;	// update banking
}

void Boid::draw(ofMatrix4x4 billboardMatrix) {
	// calculate rotation for boid to face direction of velocity
	glm::vec3 forward(0, 0, 1);  // forward direction of the boid
	glm::mat4 headRotation = rotateToVector(forward, glm::normalize(glm::vec3(velocity)));
	
	// calculate rotation for banking
	this->computeBanking(); // update banking
	glm::mat4 bankingRotation = glm::rotate(glm::mat4(1.0f), this->banking, glm::vec3(0, 0, 1));
	
	// translate to world space
	glm::mat4 rotMatrix = glm::translate(glm::mat4(1.0f), glm::vec3(this->position)) * headRotation * bankingRotation;
	
	// draw boid
	ofPushMatrix();
	ofMultMatrix(rotMatrix);
	ofRotateDeg(180, 1, 180, 0);
	frames[currentFrame]->drawFaces();
	ofPopMatrix();

	// draw sound circle
	ofPushMatrix();
	glm::vec3 forwardDir = glm::normalize(glm::vec3(velocity));
	glm::vec3 coneTipPos = position + forwardDir * radius * 0.8;
	ofMultMatrix(glm::translate(glm::mat4(1.0f), coneTipPos));
	if (soundPlayed) {
		// capture start time when the sound is played (if not already set)
		if (soundStartTime == 0.0f) {
			soundStartTime = ofGetElapsedTimef();  // mark when sound starts
		}

		// calculate alpha so circle loses transparency over 1.5 seconds
		float alpha = ofMap(ofGetElapsedTimef() - soundStartTime, 0, 1.5, 255, 0, true);

		// draw the circle
		ofMultMatrix(billboardMatrix);
		ofSetColor(ofColor::white, alpha);
		ofDisableLighting();
		ofDrawCircle(0, 0, 0, 2.5f);
		ofEnableLighting();

		// once alpha reaches 0 (after 1.5 seconds), stop drawing
		if (ofGetElapsedTimef() - soundStartTime >= 1.5f) {
			soundPlayed = false;  // reset flag after fade is complete
			soundStartTime = 0;
		}
	}
	ofPopMatrix();
}

void Boid::computeFlockingForces(glm::vec3& collision_avoidance, glm::vec3& velocity_matching, glm::vec3& centering) {
	// for each boid in neighbor list, compute accelerations
	for (Boid* b : this->neighbors) {

		// compute priority using distance thresholds
		float d = glm::distance(glm::vec3(b->position), glm::vec3(this->position));
		float k_d;
		// determine priority
		if (d < MIN_DIST_THRESHOLD) {
			k_d = 1.0f;					// full priority
		}
		else if (MIN_DIST_THRESHOLD <= d && d <= MAX_DIST_THRESHOLD) {
			k_d = (MAX_DIST_THRESHOLD - d) / (MAX_DIST_THRESHOLD - MIN_DIST_THRESHOLD);	// partial priority
		}
		else {	// d > MAX_DIST_THRESHOLD
			k_d = 0.0f;					// no priority
		}

		// compute priority using field of view threshold
		float theta1 = MIN_THETA_THRESHOLD;	// monocular field of view
		float theta2 = MAX_THETA_THRESHOLD;	// binocular field of view
		// compute vector from boid i (this) to boid j
		glm::vec3 vec_ij = b->position - this->position;
		// compute angle formed between (this) boid i's velocity and boid j
		float theta = glm::degrees(acos(glm::dot(glm::normalize(glm::vec3(velocity)), glm::normalize(vec_ij))));
		float k_theta;
		// determine priority
		if (((-theta1 / 2) <= glm::abs(theta)) && (glm::abs(theta) <= (theta1 / 2))) {
			k_theta = 1.0f;				// full priority
		}
		else if (((theta1 / 2) <= glm::abs(theta)) && (glm::abs(theta) <= (theta2 / 2))) {
			k_theta = ((theta2 / 2) - glm::abs(theta)) / ((theta2 / 2) - (theta1 / 2));
			// partial priority
		}
		else {	// glm::abs(theta) > (theta2/2)
			k_theta = 0.0f;				// no priority
		}

		// priority scaler (combining distance and field of view thresholds)
		float priority = k_theta * k_d;

		// compute distance from i to j
		float dist_ij = glm::length(vec_ij);
		// compute direction vector from i to j
		glm::vec3 dir_ij = vec_ij / dist_ij;

		collision_avoidance += priority * -(collision_avoidance_factor / dist_ij) * vec_ij;
		velocity_matching += priority * velocity_matching_factor * (b->velocity - this->velocity);
		centering += priority * centering_factor * vec_ij;
	}
}

void Boid::integrate() {
	// adjust pitch of sound according to boid's altitude
	float pitch = ofMap(position.y, -30.0f, 30.0f, 0.5, 2.0, true);

	// define acceleration forces
	glm::vec3 collision_avoidance = glm::vec3(0, 0, 0);
	glm::vec3 velocity_matching = glm::vec3(0, 0, 0);
	glm::vec3 centering = glm::vec3(0, 0, 0);

	this->computeFlockingForces(collision_avoidance, velocity_matching, centering);

	// compute acceleration prioritization
	float acc_max = 250.0f;
	float acc_remaining = acc_max;
	acceleration = glm::vec3(0, 0, 0);

	// collision avoidance
	if (glm::length(glm::vec3(collision_avoidance)) > 0) {
		acceleration += glm::min(acc_remaining, glm::length(collision_avoidance)) * glm::normalize(collision_avoidance);
		acc_remaining = acc_max - glm::length(glm::vec3(acceleration));
	}

	// velocity matching
	if (glm::length(glm::vec3(velocity_matching)) > 0) {
		acceleration += glm::min(acc_remaining, glm::length(velocity_matching)) * glm::normalize(velocity_matching);
		acc_remaining = acc_max - glm::length(glm::vec3(acceleration));
	}

	// centering
	if (glm::length(glm::vec3(centering)) > 0) {
		acceleration += glm::min(acc_remaining, glm::length(centering)) * glm::normalize(centering);
		acc_remaining = acc_max - glm::length(glm::vec3(acceleration));
	}

	// integration using euler's averaging method

	// time step
	float h = 1.0f / FRAME_RATE;

	// compute acceleration
	//acceleration = collision_avoidance + velocity_matching + centering;
	acceleration += (forces * (1.0 / mass));	// (f = ma) --> (a = f * 1/m)

	// compute new velocity
	ofVec3f newVelocity = velocity + acceleration * h;

	// compute euler's average
	ofVec3f avgVelocity = (velocity + newVelocity) * 0.5f;

	// add damping
	avgVelocity *= damping;

	// update ball position and velocity
	position += avgVelocity * h;
	velocity = avgVelocity;

	// clear forces on particle (they get re-added each step)
	forces.set(0, 0, 0);
	
	// if boids go out of bounds of the bounding box, wrap them back around
	// 
	// wrap X
	if (position.x < -BOUNDING_BOX_WIDTH) position.x += 2 * BOUNDING_BOX_WIDTH;
	if (position.x > BOUNDING_BOX_WIDTH) position.x -= 2 * BOUNDING_BOX_WIDTH;
	//
	// wrap Y
	if (position.y < -BOUNDING_BOX_HEIGHT) position.y += 2 * BOUNDING_BOX_HEIGHT;
	if (position.y > BOUNDING_BOX_HEIGHT) position.y -= 2 * BOUNDING_BOX_HEIGHT;
	//
	// wrap Z
	if (position.z < -BOUNDING_BOX_DEPTH) position.z += 2 * BOUNDING_BOX_DEPTH;
	if (position.z > BOUNDING_BOX_DEPTH) position.z -= 2 * BOUNDING_BOX_DEPTH;
}