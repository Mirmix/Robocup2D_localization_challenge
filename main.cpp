#include <stdio.h>
#include <stdlib.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <math.h>
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>
#include "robot_defs.h"
#include "controller.h"
#include "main.h"
#include <iostream>
#include <random>

/**
 * getRobotPositionEstimate()
 * This function is called by the controller to retrieve the current 
 * robot position estimate. 
 */
void getRobotPositionEstimate(RobotState &estimatePosn)
{
    // TODO: Write your procedures to set the current robot position estimate here

    //    estimatePosn.x = 0.0;
    //    estimatePosn.y = 0.0;
    //    estimatePosn.theta = 0.0;

    //position of the robot using the inertia information.

    std::vector<Particle>::iterator it = std::max_element(particles.begin(), particles.end());
    double lambda = 0.1;
    myRS.x = lambda * myRS.x + (1 - lambda) * it->rs.x;
    myRS.y = lambda * myRS.y + (1 - lambda) * it->rs.y;
    myRS.theta = it->rs.theta;
    estimatePosn = myRS;
}


void normalizeWeight(std::vector<Particle> &particles)
{
    auto min_val = *std::min_element(particles.begin(), particles.end());

    double artificial_min = 0.000001;
    double sum = 0;
    for (auto &particle : particles)
    {
        if (particle.weight < artificial_min)
            particle.weight = artificial_min;
        sum += particle.weight;
    }

    for (auto &particle : particles)
        particle.weight /= sum;
}

/**
 * motionUpdate()
 * This function is called every time the position of the robot is
 * updated. The argument passed is the relative change in position of the 
 * robot in local robot coordinates (observed by odometry model), which 
 * may be subject to noise (according to motion model parameters).
 */
void motionUpdate(RobotState delta)
{
    // TODO: Write your motion update procedures here

    double distance = sqrt(delta.x * delta.x + delta.y * delta.y);
    double absolute_theta = fabs(delta.theta);
    double rotation_variance = myRobotParams.odom_noise_rotation_from_rotation * absolute_theta + myRobotParams.odom_noise_rotation_from_translation * distance;
    double translation_variance = myRobotParams.odom_noise_translation_from_translation * distance + myRobotParams.odom_noise_translation_from_rotation * absolute_theta;
    std::default_random_engine generator;
    std::normal_distribution<double> GaussianNoiseForX(delta.x, translation_variance);
    std::normal_distribution<double> GaussianNoiseForY(delta.y, translation_variance);
    std::normal_distribution<double> GaussianNoiseForTheta(delta.theta, rotation_variance);
    for (auto &particle : particles)
    {
        double new_delta_x = GaussianNoiseForX(generator);
        double new_delta_y = GaussianNoiseForY(generator);
        double new_delta_theta = GaussianNoiseForTheta(generator);

        double theta = particle.rs.theta;
        double transformed_x = cos(theta) * delta.x - sin(theta) * delta.y;
        double transformed_y = sin(theta) * delta.x + cos(theta) * delta.y;

        particle.rs.x += transformed_x;
        particle.rs.y += transformed_y;
        particle.rs.theta += new_delta_theta;
        if (particle.rs.theta > M_PI)
            particle.rs.theta -= 2 * M_PI;
        if (particle.rs.theta < -M_PI)
            particle.rs.theta += 2 * M_PI;
    }
}

/**
 * sensorUpdate()
 * This function is called every time the robot detects one or more
 * landmarks in its field of view. The argument passed contains all 
 * marker obervations (marker index and position of marker in robot 
 * coordinates) for the current frame.
 */
void sensorUpdate(std::vector<MarkerObservation> observations)
{
    std::vector<double> unnormalized_probabilities(number_of_particles, 1);

    for (auto observation : observations)
    {
        auto landmark = myLandmarks[observation.markerIndex];
        size_t particle_index = 0;
        for (auto &particle : particles)
        {
            auto particle_rs = particle.rs;
            double estimated_distance = sqrt((landmark.x - particle_rs.x) * (landmark.x - particle_rs.x) + (landmark.y - particle_rs.y) * (landmark.y - particle_rs.y));
            double estimated_orientation = atan2((landmark.y - particle_rs.y), (landmark.x - particle_rs.x)) - particle_rs.theta;
            double distance_prob = exp(-(estimated_distance - observation.distance) * (estimated_distance - observation.distance) / (2.0 * myRobotParams.sensor_noise_distance));
            double orientation_prob = exp(-(estimated_orientation - observation.orientation) * (estimated_orientation - observation.orientation) / (2.0 * myRobotParams.sensor_noise_orientation));
            unnormalized_probabilities[particle_index] *= (distance_prob * orientation_prob);
            particle_index++;
        }
    }

    for (size_t particle_index = 0; particle_index < number_of_particles; particle_index++)
    {
        particles[particle_index].weight *= unnormalized_probabilities[particle_index];
    }
    normalizeWeight(particles);
}
/**
 * myinit()
 * Initialization function that takes as input the initial 
 * robot state (position and orientation), and the locations
 * of each landmark (global x,y coordinates).
 */

double getDoubleBetweenOnes()
{
    return (rand() * (2.0 / RAND_MAX) - 1);
}

void myinit(RobotState robotState, RobotParams robotParams,
            FieldLocation markerLocations[NUM_LANDMARKS])
{
    // TODO: Write your initialization procedures here
    myRS = robotState;
    myRobotParams = robotParams;
    for (size_t LandmarkIdx = 0; LandmarkIdx < NUM_LANDMARKS; LandmarkIdx++)
        myLandmarks.emplace_back(markerLocations[LandmarkIdx]);

    //initial particles generation
    double initial_uniform_weight = 1.0 / number_of_particles;
    Particle particle;
    double FIELD_HALF_LENGTH_IN_METERS = FIELD_LENGTH * METERS_PER_PIXEL / 2.0;
    double FIELD_HALF_WIDTH_IN_METERS = FIELD_LENGTH * METERS_PER_PIXEL / 2.0;
    for (size_t ParticleIdx = 0; ParticleIdx < number_of_particles; ParticleIdx++)
    {
        particle.weight = initial_uniform_weight;
        particle.rs.x = FIELD_HALF_LENGTH_IN_METERS * getDoubleBetweenOnes();
        particle.rs.y = FIELD_HALF_WIDTH_IN_METERS * getDoubleBetweenOnes();
        particle.rs.theta = M_PI * getDoubleBetweenOnes();
        particles.emplace_back(particle);
    }
}

/**
 * mydisplay()
 * This function is called whenever the display is updated. The controller
 * will draw the estimated robot position after this function returns.
 */
void mydisplay()
{
    // TODO: Write your drawing procedures here
    //       (e.g., robot position uncertainty representation)

    //    // Example drawing procedure
    int pixelX, pixelY;
    double globalX = 1.0, globalY = -1.0;
    const int NUM_POINTS = 8;
    const double POINT_SPREAD = 0.2;

    //// Draw cyan colored points at specified global locations on field
    glBegin(GL_POINTS);
    glColor3f(0.0, 1.0, 1.0);
    for (int i = 0; i < NUM_POINTS; i++)
    {
        global2pixel(globalX, globalY + (i * POINT_SPREAD), pixelX, pixelY);
        glVertex2i(pixelX, pixelY);
    }
    glEnd();
}

/**
 * mykeyboard()
 * This function is called whenever a keyboard key is pressed, after
 * the controller has processed the input. It receives the ASCII value 
 * of the key that was pressed.
 *
 * Return value: 1 if window re-draw requested, 0 otherwise
 */
int mykeyboard(unsigned char key)
{
    // TODO: (Optional) Write your keyboard input handling procedures here

    return 0;
}

/**
 * Main entrypoint for the program.
 */
int main(int argc, char **argv)
{
    // Initialize world, sets initial robot position
    // calls myinit() before returning
    runMainLoop(argc, argv);

    return 0;
}
