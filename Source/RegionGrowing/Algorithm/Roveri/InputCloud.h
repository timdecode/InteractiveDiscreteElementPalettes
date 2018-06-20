#pragma once

#include "Cloud.h"
#include <Eigen/Geometry>

class InputCloud
{
public:
	InputCloud(void);
	~InputCloud(void);

	Cloud cloud;
	Eigen::Vector3f center_of_mass;
	float average_samples_distance;
	float input_scale;
	float max_distance_allowed;

	// Call after adding the samples
	void initCloud();

	void processModelWithScale();
	void precomputeInputEnergy(float omega);
	std::vector<std::string> retrieveFeaturesFileNames(int is_mesh);

	std::vector<int> insideBorders;

	std::vector<std::vector<float> > precomputedInputEnergies;
	Eigen::MatrixXf precomputedInputEnergiesEigen;

	std::vector<std::vector<int> > features_indices;
	std::vector<Eigen::Vector3f> features_centers;

	std::vector<float> scales_for_features;
	std::vector<Eigen::Matrix3f> rotations_for_features;
	std::vector<Eigen::Vector3f> shift_for_features;
	//Eigen::Vector3f center_for_features;

	float target_average_distance;

	std::vector<Eigen::Vector3f> colorAttributes;
	std::vector<float> colors;


private:

	void computeCenterOfMass();
	void computeAverageSamplesDistance();
	void assignInitialZOrientation(float angle);
	void assignInitialXOrientation(float angle);
	void assignInitialYOrientation(float angle);
	void addFeaturesSamples();
	void setInitialAverageDistance();
	void loadTagAttribute();


	
	
};

inline float exp1( float x ) { // DUPLICATED IN SIMULATIONMANAGER!!!
	x = 1.0 + x / 256.0;
	x *= x; x *= x; x *= x; x *= x;
	x *= x; x *= x; x *= x; x *= x;
	return x;
}
