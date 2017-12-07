/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>
#include <random>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

    // create random distribution centered around initial position. Assume std[] has length = 3
    default_random_engine randGen;
    normal_distribution<double> distribX(x, std[0]);
    normal_distribution<double> distribY(y, std[1]);
    normal_distribution<double> distribTheta(theta, std[2]);

    num_particles = 100;
    particles = std::vector<Particle>(num_particles);
    int id = 0;
    for(auto& particle : particles)
    {
        particle.id = id;
        id++;
        particle.x = distribX(randGen);
        particle.y = distribY(randGen);
        particle.theta = distribTheta(randGen);
        particle.weight = 1;
    }
    is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

    // For motion noise, create random distribution centered around 0. Assume std_pos[] has length = 3
    default_random_engine randGen;
    normal_distribution<double> distribX(0, std_pos[0]);
    normal_distribution<double> distribY(0, std_pos[1]);
    normal_distribution<double> distribTheta(0, std_pos[2]);

    // propagate motion
    const double EEPS = 1e-6;
    for(auto& particle : particles)
    {
        //avoid division by zero
        if(yaw_rate < EEPS)
        {
            particle.x += cos(particle.theta) * delta_t * velocity;
            particle.y += sin(particle.theta) * delta_t * velocity;
            particle.theta += yaw_rate * delta_t;
        }
        else
        {
            const double vt = velocity/yaw_rate;
            const double cp = cos(particle.theta);
            const double sp = sin(particle.theta);
            particle.theta += yaw_rate * delta_t;
            particle.x += vt * (sin(particle.theta) - sp);
            particle.y += vt * (-cos(particle.theta) + cp);
        }

        // add motion noise
        particle.x += distribX(randGen);
        particle.y += distribY(randGen);
        particle.theta += distribTheta(randGen);
    }

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

    // for every observation, iterate through predicted observations and find the id of the closest one.
    for(auto& landmarkSeen : observations)
    {
        double minDist = numeric_limits<double>::max();
        int minDistId = -1;
        for(auto& landmarkPred : predicted)
        {
            double distance = dist(landmarkSeen.x, landmarkSeen.y, landmarkPred.x, landmarkPred.y);
            if(distance < minDist)
            {
                minDist = distance;
                minDistId = landmarkPred.id;
            }
        }
        landmarkSeen.id = minDistId;
    }

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

    // Steps:
    // - find predicted observations within sensor range
    // - associate those predictions with actual observations
    // - calculate weight as product of pdf(z_k; z_k*, sigma) for all observations k

    // assume measurement axes are uncorrelated, ie cov_xy = 0
    // since particle weight calculations are based on range, use std_landmark[0] for range measurement uncertainty
    // we dont use std_landmark[1], which is bearing measurement uncertainty
    const double scaledVar = -0.5/(std_landmark[0] * std_landmark[0]);

    for(auto& particle : particles)
    {
        //----------------------------
        // predictions within sensor range
        //
        // Landmark in particle coordinates: T_pl = Rt * (T_l - T_p) = Rt * (T_plm)
        //  where   T_l: landmark in map coordinates
        //          T_p: particle in map coordinates
        //          Rt: transpose of particle orientation in the map
        //          T_plm: relative position of landmark to particle in map coordinates
        //----------------------------

        // elements of rotation matrix
        const double cp = cos(particle.theta);
        const double sp = sin(particle.theta);

        std::vector<LandmarkObs> predicted; // list of predicted landmarks
        for(auto& landmark : map_landmarks.landmark_list)
        {
            // landmark relative to particle in map coordinates
            double T_plm_x = landmark.x_f - particle.x;
            double T_plm_y = landmark.y_f - particle.y;

            double distToLandmark = sqrt(T_plm_x*T_plm_x + T_plm_y*T_plm_y);
            if(distToLandmark < sensor_range)
            {
                // transform relative landmark pose to particle coordinates
                LandmarkObs prediction;
                prediction.id = landmark.id_i;
                prediction.x = cp * T_plm_x + sp * T_plm_y;
                prediction.y = -sp * T_plm_x + cp * T_plm_y;
                predicted.push_back(prediction);
            }
        } // predictions

        //----------------------------
        // associate observations with predictions
        //----------------------------

        // note: we know we only modify the landmark ID of observations. So it's ok to
        // directly modify 'observations' vector. Work with a copy if observations are
        // modified in any other way
        dataAssociation(predicted, observations);

        //----------------------------
        // calculate weight of the particle
        //
        //  weight = PROD[ {1/sqrt(2 * pi * det(cov))} * exp(0.5 * d_i^T * cov^-1 * d_i) ]
        //          where   PROD denotes cumulative product over all i
        //                  d_i = relative pose of landmark = observation - prediction
        //                  cov = measurement covariance
        //                  det(cov) = determinant of measurement covariance
        // we can safely ignore the first term, which is a system constant (does not change for every observation)
        // as the scale will not effect particle resampling by weight. Therefore the weight is a product of
        // exponentials alone.
        //----------------------------
        particle.weight = 1; //initialise
        for(auto& landmarkSeen : observations)
        {
            // find the prediction for this observation
            for(auto& landmarkPred : predicted)
            {
                if( landmarkPred.id == landmarkSeen.id )
                {
                    // update particle weight by accumulating observation weight
                    const double dx = landmarkSeen.x - landmarkPred.x;
                    const double dy = landmarkSeen.y - landmarkPred.y;
                    const double d = scaledVar * (dx * dx + dy * dy);
                    particle.weight *= exp(d);
                }
            }
        } // weight update
    } // for each particle
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

    // create a vector of weights alone
    std::vector<double> weights;
    for(const auto& particle : particles)
    {
        weights.push_back(particle.weight);
    }

    // create a random number generator that chooses a particle in proportion to its weight
    std::random_device randDev;
    std::mt19937 randGen(randDev());
    std::discrete_distribution<> distrib(weights.begin(), weights.end());

    std::vector<Particle> selectedParticles;
    for(int i = 0; i < particles.size(); ++i)
    {
        selectedParticles.push_back( particles[distrib(randGen)] );
    }

    particles = selectedParticles;
}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
 	particle.sense_x = sense_x;
 	particle.sense_y = sense_y;

 	return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
