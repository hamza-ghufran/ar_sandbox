#include "Boid.h"


Boid::Boid(int num,int x, int y, int xbound, int ybound,
        int     mboundaryPadding    ,
        float   mmaxSpeed           ,
        float   mmaxForce           ,
        float   mflockSepWeight     ,
        float   mflockAliWeight     ,
        float   mflockCohWeight     ,
        float   mflockSepRadius     ,
        float   mflockAliRadius     ,
        float   mflockCohRadius     )
{
    id = num;
    loc.setval(x,y);
    prev_loc.setval(x,y);
	vel.setval(0,0);
	acc.setval(0,0);
    r = 3.0;
    orient = 0;
    endCorner.setval(xbound,ybound);
    reachedDestination = false;
    hitObstacle = false;

    boundaryPadding =   mboundaryPadding;
    maxSpeed        =   mmaxSpeed      ;
    maxForce        =   mmaxForce      ;
    flockSepWeight  =   mflockSepWeight;
    flockAliWeight  =   mflockAliWeight;
    flockCohWeight  =   mflockCohWeight;
    flockSepRadius  =   mflockSepRadius;
    flockAliRadius  =   mflockAliRadius;
    flockCohRadius  =   mflockCohRadius;

}

// Method to update location
void Boid::update(vector<Boid> &boids) {

	flock(boids);

    vel += acc;   // Update velocity
    vel.x = clamp(vel.x, -maxSpeed, maxSpeed);  // Limit speed
	vel.y = clamp(vel.y, -maxSpeed, maxSpeed);  // Limit speed
	prev_loc = loc;
    loc += vel;
    acc.setval(0,0);  // Resetval accelertion to 0 each cycle
    orient = (float)atan2(vel.y,vel.x) * 180/PI;

    boundCheck(boundaryPadding);


}

void Boid::seek(Vec2f target,float weight,bool normalize) {
    acc += steer(target,normalize)*weight;
}

void Boid::avoid(Vec2f target,float weight,bool normalize) {
    acc -= steer(target,normalize)*weight;
}

void Boid::boundCheck(int padding) {

    if(loc.x>endCorner.x-padding)
    {
        loc.x=endCorner.x-padding;
        vel.x=-vel.x;

    }

    else if(loc.x<0+padding)
     {
         loc.x=0+padding;
         vel.x=-vel.x;
     }

    if(loc.y>endCorner.y-padding)
    {
        loc.y=endCorner.y-padding;
        vel.y=-vel.y;
    }

    else if(loc.y<0+padding)
    {
        loc.y=0+padding;
        vel.y=-vel.y;
    }


}


// A method that calculates a steering vector towards a target
Vec2f Boid::steer(Vec2f target,bool normalize) {
    Vec2f steer;  // The steering vector
    Vec2f desired = target - loc;  // A vector pointing from the location to the target

	float d = target.distance(loc); // Distance from the target is the magnitude of the vector


	// If the distance is greater than 0, calc steering (otherwise return zero vector)
    if (d > 0) {

		if (normalize)
		{
			desired /= d; // Normalize desired

			desired *= maxSpeed;
		}

		// Steering = Desired minus Velocity
		steer = desired - vel;
		steer.x = clamp(steer.x, -maxForce, maxForce); // Limit to maximum steering force
		steer.y = clamp(steer.y, -maxForce, maxForce);

    }
    return steer;
}


void Boid::flock(vector<Boid> &boids) {
	Vec2f sep = separate(boids);
	Vec2f ali = align(boids);
	Vec2f coh = cohesion(boids);

	// Arbitrarily weight these forces
	sep *= flockSepWeight;
	ali *= flockAliWeight;
	coh *= flockCohWeight;

	acc += sep + ali + coh;
}

/*
 postscript
 */
bool Boid::isHit(int x, int y, int radius) {

    if(pow((x-loc.x),2)+pow((y-loc.y),2) < pow(radius,2)) {
        return true;
    }
    return false;
}

// Separation
// Method checks for nearby boids and steers away
Vec2f Boid::separate(vector<Boid> &boids) {
    float desiredseparation = flockSepRadius;
    Vec2f steer;
    int count = 0;

    // For every boid in the system, check if it's too close
    for (int i = 0 ; i < boids.size(); i++) {
		Boid &other = boids[i];

		float d = loc.distance(other.loc);

		// If the distance is greater than 0 and less than an arbitrary amount (0 when you are yourself)
		if ((d > 0) && (d < desiredseparation)) {
			// Calculate vector pointing away from neighbor
			Vec2f diff = loc - other.loc;
			diff /= d;			// normalize
			diff /= d;        // Weight by distance
			steer += diff;
			count++;            // Keep track of how many
		}
    }
    // Average -- divide by how many
    if (count > 0) {
		steer /= (float)count;
    }


    // As long as the vector is greater than 0
	//float mag = sqrt(steer.x*steer.x + steer.y*steer.y);

	float mag = steer.length();
    if (mag > 0) {
		// Implement Reynolds: Steering = Desired - Velocity
		steer /= mag;
		steer *= maxSpeed;
		steer -= vel;
		steer.x = clamp(steer.x, -maxForce, maxForce);
		steer.y = clamp(steer.y, -maxForce, maxForce);
    }
    return steer;
}

// Alignment
// For every nearby boid in the system, calculate the average velocity
Vec2f Boid::align(vector<Boid> &boids) {
    float neighbordist = flockAliRadius;
    Vec2f steer;
    int count = 0;
    for (int i = 0 ; i < boids.size(); i++) {
		Boid &other = boids[i];

		float d = loc.distance(other.loc);
		if ((d > 0) && (d < neighbordist)) {
			steer += (other.vel);
			count++;
		}
    }
    if (count > 0) {
		steer /= (float)count;
    }

    // As long as the vector is greater than 0
	float mag = steer.length();
    if (mag > 0) {
		// Implement Reynolds: Steering = Desired - Velocity
		steer /= mag;
		steer *= maxSpeed;
		steer -= vel;
		steer.x = clamp(steer.x, -maxForce, maxForce);
		steer.y = clamp(steer.y, -maxForce, maxForce);
    }
    return steer;
}

// Cohesion
// For the average location (i.e. center) of all nearby boids, calculate steering vector towards that location
Vec2f Boid::cohesion(vector<Boid> &boids) {
    float neighbordist = flockCohRadius;
    Vec2f sum;   // Start with empty vector to accumulate all locations
    int count = 0;
    for (int i = 0 ; i < boids.size(); i++) {
		Boid &other = boids[i];
		float d = loc.distance(other.loc);
		if ((d > 0) && (d < neighbordist)) {
			sum += other.loc; // Add location
			count++;
		}
    }
    if (count > 0) {
		sum /= (float)count;
		return steer(sum);  // Steer towards the location
    }
    return sum;
}


float Boid::dist(Vec2f v1,Vec2f v2)
{
    return v1.distance(v2);
}

float Boid::clamp(float val, float minval, float maxval)
{
    if(val<minval)
        return minval;
    else if (val>maxval)
        return maxval;
    else
        return val;
}

void Boid::draw()
{

}

void Boid::updateSimParams(
        float 	mmaxSpeed 			,
		float 	mmaxForce 			,
		float 	mflockSepWeight 	,
		float 	mflockAliWeight 	,
		float 	mflockCohWeight 	,
		float 	mflockSepRadius 	,
		float 	mflockAliRadius 	,
		float 	mflockCohRadius 	)
		{
		    maxSpeed            = mmaxSpeed;
            maxForce 			= mmaxForce;
            flockSepWeight 	    = mflockSepWeight;
            flockAliWeight 	    = mflockAliWeight;
            flockCohWeight 	    = mflockCohWeight;
            flockSepRadius 	    = mflockSepRadius;
            flockAliRadius 	    = mflockAliRadius;
            flockCohRadius      = mflockCohRadius;
		}
