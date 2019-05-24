/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 *
 * Project Kidnapped Vehicle-Term 2 Self Driving Car Engineer nanodegree
 * Modified by Srujan Korlakunta on 04 March 2019
 */
#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>
#include <sstream>

#include "helper_functions.h"

using std::uniform_real_distribution;
using std::uniform_int_distribution;
using std::numeric_limits;
using std::string;
using std::vector;
using std::normal_distribution;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
   std::default_random_engine generator;
   num_particles = 100;  // TODO: Set the number of particles
  
  //generating the normal districutions for the measurements x, y, theta with their respective standard deviations
   normal_distribution<double> dist_x(x, std[0]);
   normal_distribution<double> dist_y(y, std[1]);
   normal_distribution<double> dist_theta(theta,std[2]);
  
  //Sampling from a gaussian distribution, centered around the passed GPS Measurements
  for (int i = 0; i < num_particles; i++) {
  //initializing an object of type particle  
  Particle p;
  p.id = i;
  p.weight = 1.0;
  p.x = dist_x(generator);
  p.y = dist_y(generator);
  p.theta = dist_theta(generator);
    
  particles.push_back(p);
  
	}  
  is_initialized = true; // indicates that the particle filter is successfully initialized
}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */
  
  std::default_random_engine generator;
  // Generating the normal distributions for sensor noise
  std::normal_distribution<double> dist_x(0.0, std_pos[0]);
   std::normal_distribution<double> dist_y(0.0, std_pos[1]);
   std::normal_distribution<double> dist_theta(0.0, std_pos[2]);
  
  for (int i = 0; i < num_particles; i++) {  

   // calculating the new state with the help of velocity and yaw rate    
    //Checking if the yaw rate is negligible in which case only velocity has an impact on the new state
    //also theta doesnot change if the yaw rate is not changing 
    
    if (fabs(yaw_rate) < 0.00001) {  
      particles[i].x += velocity * delta_t * cos(particles[i].theta);
      particles[i].y += velocity * delta_t * sin(particles[i].theta);
    }     
    //if yaw rate is changing, on using the formulas shown in the lesson new state of the particle can be calculated
    else {
      particles[i].x += velocity / yaw_rate * (sin(particles[i].theta + yaw_rate*delta_t) - sin(particles[i].theta));
      particles[i].y += velocity / yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate*delta_t));
      particles[i].theta += yaw_rate * delta_t;
    }
     
  // adding random gaussian noise with a normal distribution which has the mean of the passed measurement and its standard deviation 
  particles[i].x += dist_x(generator);
  particles[i].y += dist_y(generator);
  particles[i].theta += dist_theta(generator);       
  }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */
  
  //using the nearest neighbor algorithm to find association between the obervations and the actual landmark positions
  
  for (unsigned int i = 0; i < observations.size(); i++){
    // initializing the placeholder with an impossible value for map_id
    int map_id = -100; 
    // initializing the minimum distance to the maximum possible value
    double minimum_dist = std::numeric_limits<double>::max();
  	LandmarkObs Obs = observations[i];
      for(unsigned int j = 0; j < predicted.size(); j++){
        LandmarkObs Pred = predicted[j];
        // getting the distance between current observation and predicted landmarks
      	double current_dist = dist(Obs.x, Obs.y, Pred.x, Pred.y);                
        if (current_dist < minimum_dist) {
        	minimum_dist = current_dist;
        	map_id = Pred.id;
        }
    // Updating the observation id with the map_id obtained by using the nearest neighbor algorithm
    observations[i].id = map_id;
      }
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian 
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */
  
  //declaring a normalizing factor to later normalize all the particle weights so that they add up to 1
  std::default_random_engine generator;  
  double normalizing_factor = 0.0;
  //Running a loop through all the particles
  for ( int i = 0; i < num_particles; i++) {   
    // initializing a vector to be assigned to the map landmarks predicted to be in the sensor range   
    vector<LandmarkObs> predictions_in_Range;
	
   float particle_x = particles[i].x;
   float particle_y = particles[i].y;
   double particle_theta = particles[i].theta;
   int count = 0;
     //running the loop through all the landmarks
     for (unsigned int j = 0; j < map_landmarks.landmark_list.size(); j++){
       
       float map_landmark_x = map_landmarks.landmark_list[j].x_f;
   	   float map_landmark_y = map_landmarks.landmark_list[j].y_f;
       int map_landmark_id = map_landmarks.landmark_list[j].id_i;
       
         //to check if the current in-loop map landmark lies in the sensor range of the particle
       // double diff_x = particle_x-map_landmark_x;
      //  double diff_y = particle_y-map_landmark_y;
         // if the calculated distance between the particle and the current map landmark is less than or equal to sensor range

       if(fabs(map_landmark_x - particle_x) <= sensor_range && fabs(map_landmark_y - particle_y) <= sensor_range) {
           // adding this prediction to the vector of in-range predictions
          predictions_in_Range.push_back(LandmarkObs{ map_landmark_id, map_landmark_x, map_landmark_y}); 
         count = count + 1;//to count the number of predictions in sensor range
         }          
     }
    
    //Transforming the observations from the vehicle coordinates to the map coordinates 
    vector<LandmarkObs> Trsnf_Observations;
    for (unsigned int k = 0; k < observations.size(); k++){       
       double Trsnf_Obs_x = cos(particle_theta)*observations[k].x - sin(particle_theta)*observations[k].y + particle_x;
       double Trsnf_Obs_y = sin(particle_theta)*observations[k].x + cos(particle_theta)*observations[k].y + particle_y;
       Trsnf_Observations.push_back(LandmarkObs{ observations[k].id, Trsnf_Obs_x, Trsnf_Obs_y });
    }
    
    // calling the previous dataAssociation function on the transformed observartions and the in range predictions
    dataAssociation(predictions_in_Range, Trsnf_Observations);
    
    // reinitializing the particle weight to 1
    particles[i].weight = 1.0;
   
    //calculating the weights
    for (unsigned int l = 0; l < Trsnf_Observations.size(); l++) {       
        double Trsnf_Observations_x = Trsnf_Observations[l].x;
        double Trsnf_Observations_y = Trsnf_Observations[l].y;
        int Trsnf_Observations_id = Trsnf_Observations[l].id;
        double multi_var_prob = 1.0;
        
          for (unsigned int m = 0; m < predictions_in_Range.size(); m++) {          
               int predictions_in_Range_id = predictions_in_Range[m].id;                 
               double predictions_in_Range_x = predictions_in_Range[m].x;
               double predictions_in_Range_y = predictions_in_Range[m].y;  
            //checking if the prediction id mateches with the transformed map coordinate observation
            if (predictions_in_Range_id == Trsnf_Observations_id){
              
              double gauss_norm;
  			  gauss_norm = 1 / (2 * M_PI * std_landmark[0] * std_landmark[1]);
              
              // calculating the exponent
             double exponent;
             exponent = (pow(Trsnf_Observations_x - predictions_in_Range_x, 2) / (2 * pow(std_landmark[0], 2))) + (pow(Trsnf_Observations_y - predictions_in_Range_y, 2) / (2 * pow(std_landmark[1], 2)));
    
             // calculate weight using normalization terms and exponent
             
             multi_var_prob = gauss_norm * exp(-exponent);              
             
           //calculating the weight of each particle using the multivariate gaussian distribution
          // product of this obersvation weight and the total previous observations weights
              particles[i].weight *= multi_var_prob;
              
                 }
          } 
    }
    //normalizing factor is the sum of the weights of the particles
	normalizing_factor += particles[i].weight;
    //std::cout << "particles[i].weight = " << particles[i].weight ; 
     }  
  //normalizing the weights of each particle by dividing with the normalizing factor  
   for (unsigned int n = 0; n < particles.size(); n++) {
    particles[n].weight /= normalizing_factor;  
    
  	}
  }


void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
    std::default_random_engine generator;
  //initializing a vector to hold the resampled particles
    vector<Particle> resampled_particles; 
  
   // storing the current weights in a vector
// Get weights and max weight.
  vector<double> weights;
  double max_weight = numeric_limits<double>::min();
  for(int i = 0; i < num_particles; i++) {
    weights.push_back(particles[i].weight);
    if ( particles[i].weight > max_weight ) {
      max_weight = particles[i].weight;
    }
  }  
  // generating a random starting index for the resampling wheel
   uniform_int_distribution<int> uniintdist(0, num_particles-1);  
  // generating a uniform random ristribution for the weights of the particles
  uniform_real_distribution<double> unirealdist(0.0, max_weight);  
  double beta = 0.0;
  int index = uniintdist(generator);
  
  // looping through the resampling wheel  
  for(int j = 0; j < num_particles; j++) {
    beta += unirealdist(generator) * 2.0;
    while( beta > weights[index]) {
      beta -= weights[index];
      index = (index + 1) % num_particles;
    }
    // appending the resampled particles to the vector
    resampled_particles.push_back(particles[index]);
  }
  //replacing the initial particles vector, with the resampled particles
  particles = resampled_particles;   
}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}