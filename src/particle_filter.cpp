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

#include "particle_filter.h"
#include "helper_functions.h"

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

	std::default_random_engine gen;
	std::normal_distribution<double> dist_x(x, std[0]);
	std::normal_distribution<double> dist_y(y, std[1]);
	std::normal_distribution<double> dist_theta(theta, std[2]);

	num_particles = 100;
	for(int i=0; i<num_particles; i++)
	{
		Particle particle;
		particle.id = i;
		particle.x = dist_x(gen);
		particle.y = dist_y(gen);
		particle.theta = dist_theta(gen);
		particle.weight = 1.0;
		particles.push_back(particle);		
	}
	is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	std::default_random_engine gen;
	std::normal_distribution<double> dist_x(0.0, std_pos[0]);
	std::normal_distribution<double> dist_y(0.0, std_pos[1]);
	std::normal_distribution<double> dist_theta(0.0, std_pos[2]);

	for(int i=0; i<num_particles; i++)
	{
		Particle particle = particles[i];
		double new_x = 0.0;
		double new_y = 0.0;
		double new_theta = 0.0;

		if (fabs(yaw_rate) > 0.001)
		{
			new_x = particle.x + (velocity / yaw_rate) * (sin(particle.theta + yaw_rate * delta_t) - sin(particle.theta));
			new_y = particle.y + (velocity / yaw_rate) * (-cos(particle.theta + yaw_rate * delta_t) + cos(particle.theta));
			new_theta = particle.theta + yaw_rate * delta_t;
		}
		else
		{
			new_x = particle.x + velocity * delta_t * cos(particle.theta);
			new_y = particle.y + velocity * delta_t * sin(particle.theta);;
			new_theta = particle.theta;
		}
		
		particles[i].x = new_x + dist_x(gen);
		particles[i].y = new_y + dist_y(gen);
		particles[i].theta = new_theta + dist_theta(gen);
	}

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	
	for (int i = 0; i<observations.size(); i++)
	{
		LandmarkObs obs = observations[i];
		double min_dist = 10000.0;
		for (int j = 0; j<predicted.size(); j++)
		{
			LandmarkObs pred = predicted[j];
			//Only consider landmarks within sensor range
			if (pred.id != -1)
			{
				double dist_i_j = dist(pred.x, pred.y, obs.x, obs.y);
				if (dist_i_j < min_dist)
				{
					observations[i].id = pred.id;
					min_dist = dist_i_j;
				}
			}

		}
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
	//   3.33. Note that you'll need to switch the minus sign in that equation to a plus to account 
	//   for the fact that the map's y-axis actually points downwards.)
	//   http://planning.cs.uiuc.edu/node99.html
	
	weights.clear();
	weights_sum = 0.0;
	//Go through each particle and update weights
	for (int i = 0; i < num_particles; i++)
	{
		Particle particle = particles[i];

		//Use sensor range to get the landmarks within range
		//mapdata resize for efficient lookuo
		LandmarkObs defaultLandmark;
		defaultLandmark.id = -1;
		map_data.resize(42, defaultLandmark);
		for (int j = 0; j < map_landmarks.landmark_list.size(); j++)
		{
			LandmarkObs lmark;
			lmark.id = map_landmarks.landmark_list[j].id_i;
			lmark.x = map_landmarks.landmark_list[j].x_f;
			lmark.y = map_landmarks.landmark_list[j].y_f;
			double dist_obs_pred = dist(lmark.x, lmark.y, particle.x, particle.y);
			if(dist_obs_pred <= sensor_range)
			{
				//map_data.push_back(lmark);
				map_data[lmark.id - 1] = lmark;//-1 since id goes from 1 to 42 and arr index is 0 to 41
			}

		}

		//Create vector of observations in map co-ordinates
		std::vector<LandmarkObs> map_observations;
		//Transform observation to map coordinate system
		//Apply rotation and then translation on each observation
		for(int j=0; j<observations.size();j++)
		{
			LandmarkObs obs = observations[j];
			LandmarkObs obs_map;
			obs_map.x = cos(particle.theta)*obs.x - sin(particle.theta)*obs.y + particle.x;
			obs_map.y = sin(particle.theta)*obs.x + cos(particle.theta)*obs.y + particle.y;
			obs_map.id = -1;// obs.id; Using -1 to indicate to check for unassigned state later
			map_observations.push_back(obs_map);
		}
		//do the data association for the observations
		dataAssociation(map_data, map_observations);
		//Compute particle weight
		particles[i].weight *= computeMultiVariateGaussian(map_observations, map_data, std_landmark);
		weights.push_back(particles[i].weight);
		weights_sum += particles[i].weight;
	}

}

double ParticleFilter::computeMultiVariateGaussian(std::vector<LandmarkObs> map_observations, std::vector<LandmarkObs> landmarks, double std_landmarks[])
{
	double prob = 1.0;
	double den = 2 * M_PI * std_landmarks[0] * std_landmarks[1];
	if(den < 0.001)
	{
		std::cout << "Error - Division by zero in multi variate gaussian computation";
	}
	for(int i=0; i<map_observations.size(); i++)
	{
		LandmarkObs map_obs = map_observations[i];
		
		//check to verify if map_obs id is meaningful before you do indexing
		if (map_obs.id == -1 || landmarks[map_obs.id - 1].id == -1)
		{
			std::cout << "Error in association logic";
		}
		LandmarkObs lmark = landmarks[map_obs.id - 1]; //note -1 because id starts from 1
		
		
		//compute prob
		double num1 = ((map_obs.x - lmark.x) * (map_obs.x - lmark.x)) / (2 * std_landmarks[0] * std_landmarks[0]);
		double num2 = ((map_obs.y - lmark.y) * (map_obs.y - lmark.y)) / (2 * std_landmarks[1] * std_landmarks[1]);
		double num = exp(-(num1 + num2));
		prob *=  num / den;
	}

	return prob;
}


void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	std::default_random_engine generator;
	
	//Normalize the weights?
	//Do not normalize if weights sum is too small
	/*if (weights_sum > 0.1)
	{
		for (int i = 0; i < num_particles; i++)
		{
			weights[i] = weights[i] / weights_sum;
		}
	}*/

	std::discrete_distribution<int> distribution(weights.begin(), weights.end());
	std::vector<Particle> newParticles;
	for (int i = 0; i<num_particles; i++)
	{
		Particle newParticle;
		int num = distribution(generator);
		newParticle = particles[num];
		//Important: Re-initialize the new particle weight to 1.0
		newParticle.weight = 1.0;
		newParticles.push_back(newParticle);
	}
	particles = newParticles;
}

void ParticleFilter::write(std::string filename) {
	// You don't need to modify this file.
	std::ofstream dataFile;
	dataFile.open(filename, std::ios::app);
	for (int i = 0; i < num_particles; ++i) {
		dataFile << particles[i].x << " " << particles[i].y << " " << particles[i].theta << "\n";
	}
	dataFile.close();
}
