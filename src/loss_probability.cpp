#include "loss_probability.h"
#include <random>
#include <chrono>

using namespace std;
using namespace ros;
using namespace XmlRpc;
using namespace Eigen;

void LossProbability::initParams (XmlRpcValue &params)
{
	VectorXd statePrior(MODEL_STATES_N);
	MatrixXd errorPrior(MODEL_STATES_N, MODEL_STATES_N);

	agentsNo = int (params["agents_no"]);
	predictionInterval = paramDouble (params["prediction_interval"]);

	statePrior = paramMatrix (params["prior"]["state"], statePrior.rows (), 1);
	errorPrior = paramMatrix (params["prior"]["error"], errorPrior.rows (), errorPrior.cols ());

	model = buildModel (params["kalman_model"]);

	kalman.setModel (model);
	kalman.initFilter (statePrior, errorPrior);

	visibilityMatrices.resize (agentsNo);

	// Setup flags
	for (int i = 0; i < agentsNo; i++) {
		flags.addFlag (i);
	}
}

KalmanModel LossProbability::buildModel (XmlRpcValue &params)
{
	MatrixXd A(MODEL_STATES_N, MODEL_STATES_N);
	MatrixXd B(MODEL_STATES_N, MODEL_INPUTS_N);
	MatrixXd C(MODEL_OUTPUTS_N, MODEL_STATES_N);
	MatrixXd D(MODEL_OUTPUTS_N, MODEL_INPUTS_N);
	double beta, sigma;
	double sampleTime;

	sampleTime = paramDouble(params["sample_time"]);
	beta = paramDouble(params["beta"]);
	sigma = paramDouble(params["sigma"]);

	A << 1, sampleTime, 0, 0,
		 0, 1, 0,		   0,
		 0, 0, 1, sampleTime,
		 0, 0, 0,		   1;
	B << beta*pow(sampleTime,2)/2, 0, 0, 0,
		 beta*sampleTime/2, 0, 0, 0,
		 0, beta*pow(sampleTime,2)/2, 0, 0,
		 0, beta*sampleTime/2, 0, 0;
	C << 1, 0, 0, 0,
		 0, 0, 1, 0;
	D << 0, 0, sigma,     0,
		 0, 0,     0, sigma;

	return KalmanModel (A, B, C, D, sampleTime);
}

int LossProbability::getAgentsNo() {
	return agentsNo;
}

void LossProbability::updateVisibility (const opt_view::SparseMatrixInt &matrixMsg) {
	visibilityMatrices[matrixMsg.agentId].fromMsg (matrixMsg);
	flags.set (matrixMsg.agentId);
}

void LossProbability::updateTargetPose (const Isometry3d &newPose, const Vector2d &vel)
{
	default_random_engine generator(std::chrono::system_clock::now().time_since_epoch().count());
	normal_distribution<double> distribution(0,0.01);
	Vector2d xy = newPose.translation ().head<2> ();
	Vector2d noise = Vector2d (distribution(generator), distribution(generator));
	poseGroundTruth = xy;
	velGroundTruth = vel;

	kalman.filter (xy);
}

void LossProbability::sumVisibilityMatrices (VisibilityMatrix &totalView)
{
	totalView = visibilityMatrices[0];
	for (int i = 1; i < agentsNo; i++) {
		totalView = totalView + visibilityMatrices[i];
	}
}

double normPdf (const Eigen::VectorXd &x, const Eigen::VectorXd &mean, const Eigen::MatrixXd &sigma)
{
	double n = x.rows();
	double sqrt2pi = std::sqrt(2 * M_PI);
	double quadform  = (x - mean).transpose() * sigma.inverse() * (x - mean);
	double norm = std::pow(sqrt2pi, - n) *
			std::pow(sigma.determinant(), - 0.5);

	return norm * exp(-0.5 * quadform);
}
#define POS_VAL(v) (Vector2d (v(0), v(2)))
#define POS_ERR(e) (Matrix2d (e(0,0), e(0,2), e(2, 0), e(2,2)))

Matrix2d getPosErr (const MatrixXd &err) {
	Matrix2d ret;
	ret << err(0,0), err(0, 2),
		   err(2,0), err(2, 2);
	return ret;
}

double LossProbability::computeProbability (const VisibilityMatrix &totalView, const Measure &prediction)
{
	Region region = totalView.getRegion ();
	Vector2d currPos;
	int h, k;
	double visibleProbability = 0;
	debugView.clear ();
	debugView.setRegion (region);
	for (int i = 0; i < totalView.count (); i++) {
		h = totalView.getElement (i).row ();
		k = totalView.getElement (i).col ();
		currPos = region(h, k);
		double val = normPdf (currPos, POS_VAL (prediction.value),
										getPosErr(prediction.error));

		visibleProbability += val * region.cellSize*region.cellSize;
		//if (val > 0.01) {
			cout << currPos(0) << " " << currPos(1) << "\n" << val << endl << endl;
			cout << prediction.value << endl << prediction.error << endl;
		//}
		debugView.set (h, k, round (val * 100000));
	}
	if (visibleProbability > 1)
		visibleProbability = 1;
	Vector2i ii = region.indicesFromWorld (POS_VAL (prediction.value));
	debugView.set (ii(0), ii(1), 100);


	return 1 - visibleProbability;
}

LossProbability::LossProbability() {
}

double LossProbability::compute ()
{
	Measure prediction;
	double probability;

	if (!isReady ())
		return -1;

	prediction = kalman.predict (predictionInterval);
	totalView.clear ();
	sumVisibilityMatrices (totalView);
	probability = computeProbability (totalView, prediction);

	flags.setProcessed ();

	return probability;
}

bool LossProbability::isReady() {
	return flags.isReady ();
}









































