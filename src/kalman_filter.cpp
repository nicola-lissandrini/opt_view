#include "kalman_filter.h"
#include "common.h"
#include <unsupported/Eigen/MatrixFunctions>

using namespace Eigen;

KalmanModel::KalmanModel (Eigen::MatrixXd _A, Eigen::MatrixXd _B, Eigen::MatrixXd _C, Eigen::MatrixXd _D, double _sampleTime)
{
	nStates = _A.rows ();
	nInputs = _B.cols ();
	nOutputs = _C.rows ();

	assert (_A.cols () == _A.rows ());
	assert (_B.rows () == nStates);
	assert (_C.cols () == nStates);
	assert (_D.cols () == nInputs && _D.rows () == nOutputs);

	R = _D * _D.transpose ();
	Q = _B * _B.transpose ();

	assert ((_B * _D.transpose ()).isZero (0) && "Correlation between input and output noise is not supported");

	A = _A;
	B = _B;
	C = _C;
	D = _D;

	sampleTime = _sampleTime;
}

KalmanFilter::KalmanFilter()
{
}

void KalmanFilter::setModel (const KalmanModel &newModel) {
	model = newModel;
}

void KalmanFilter::initFilter (VectorXd statePrior, MatrixXd errorPrior)
{
	predicted.value = statePrior;
	predicted.error = errorPrior;
}

Measure KalmanFilter::predict (int steps)
{
	Measure prediction;

	prediction.value = filtered.value;
	prediction.error = filtered.error;

	for (int k = 0; k < steps; k++) {
		prediction.value = model.A * prediction.value;
		prediction.error = model.A * prediction.error * model.A.transpose () + model.Q;
	}

	return prediction;
}

Measure KalmanFilter::predict (double secs) {
	return predict ((int)floor (secs / model.sampleTime));
}

void KalmanFilter::computeInnovation (VectorXd newMeasure)
{
	innovation.value = newMeasure - model.C * predicted.value;
	innovation.error = model.R + model.C * predicted.error * model.C.transpose ();
}

void KalmanFilter::predictionStep () {
	predicted = predict (1);
}

MatrixXd KalmanFilter::getGain () {
	return predicted.error * model.C.transpose () * innovation.error.inverse ();
}

Measure KalmanFilter::filter (VectorXd newMeasure)
{
	computeInnovation (newMeasure);

	filtered.value = predicted.value + getGain () * innovation.value;
	filtered.error = predicted.error - predicted.error * model.C.transpose() * innovation.error.inverse () * model.C * predicted.error;

	predictionStep ();
	return filtered;
}

Measure KalmanFilter::getFiltered() const {
	return filtered;
}

