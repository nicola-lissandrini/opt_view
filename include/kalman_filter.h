#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <Eigen/Core>

struct Measure
{
	Eigen::VectorXd value;
	Eigen::MatrixXd error;
};

struct KalmanModel
{
	int nStates;
	int nInputs;
	int nOutputs;
	double sampleTime;
	Eigen::MatrixXd A, B, C, D;
	Eigen::MatrixXd Q, R;

	KalmanModel (Eigen::MatrixXd _A,
				 Eigen::MatrixXd _B,
				 Eigen::MatrixXd _C,
				 Eigen::MatrixXd _D,
				 double _sampleTime = 1);
	KalmanModel () {}
};

class KalmanFilter
{
	KalmanModel model;

	Measure predicted, filtered;
	Measure lastOutput, innovation;

	void predictionStep ();
	void computeInnovation (Eigen::VectorXd newMeasure);
	Eigen::MatrixXd getGain();

public:
	KalmanFilter ();

	void setModel (const KalmanModel &newModel);
	void initFilter (Eigen::VectorXd statePrior,
					 Eigen::MatrixXd errorPrior);

	Measure predict (int step);
	Measure predict (double secs);

	Measure filter(Eigen::VectorXd newMeasure);

	Measure getFiltered ();

};

#endif // KALMAN_FILTER_H
