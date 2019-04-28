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

void LossProbability::updatePose (const Isometry3d &newPose, const Vector2d &vel)
{
	default_random_engine generator(std::chrono::system_clock::now().time_since_epoch().count());
	normal_distribution<double> distribution(0,0.01);
	Vector2d xy = newPose.translation ().head<2> ();
	Vector2d noise = Vector2d (distribution(generator), distribution(generator));
	poseGroundTruth = xy;
	velGroundTruth = vel;

	cout << "Noise\n" << noise << endl << endl;

	kalman.filter (xy + noise);
}

double LossProbability::compute ()
{
	Measure prediction, filtered;

	if (!isReady ())
		return -1;

	prediction = kalman.predict (predictionInterval);
	filtered = kalman.getFiltered ();

	FILE *fp = fopen ("/home/nicola/prediction","w");
	fprintf (fp, "%lg, %lg\n%lg, %lg\n%lg, %lg\n", prediction.value(0), prediction.value(2),
												   prediction.error(0,0), prediction.error(0,2),
												   prediction.error(2,0), prediction.error(2,2));

	fclose (fp);
	cout << "Filtered\n" << filtered.value << "\nFiltered error\n" << filtered.error << endl;
	cout << "Predicted\n" << prediction.value << "\nPredicted error\n" << prediction.error << endl;


	flags.setProcessed ();

	return 0;
}

bool LossProbability::isReady() {
	return flags.isReady ();
}








































