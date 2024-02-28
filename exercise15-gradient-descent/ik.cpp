#include <iostream>
#include "fk.h"

using namespace std;

double distanceToGoal(Vector3d goalPosition, MatrixXd jointVariables);
double sample(Vector3d goal, MatrixXd jointVariables, int joint, double delta);

int main(int argc, char** argv)
{
  // Algorithm parameters
  const int MAX_ITERATIONS = 10000;
  const double DELTA = 0.001;
  const double DAMPING = 0.001;
  const double TOLERANCE = 0.0001;

  // Starting joint states
  const double BIAS_BASE = 0;
  const double BIAS_SHOULDER = -0.5;
  const double BIAS_ELBOW = 0.5;
  const double BIAS_WRIST = 0;

  // Print usage if not enough arguments
  if (argc < 4)
  {
    cout << "Usage: " << argv[0] << " <x> <y> <z>" << endl;
    return 1;
  }

  // Parse goal position
  Vector3d goal;
  goal << stod(argv[1]), stod(argv[2]), stod(argv[3]);

  bool verbose = argc > 4 && string(argv[4]) == "--verbose";

  cout << "Inverse Kinematics (Gradient Descent)" << endl << endl;
  cout << "Given Pose ["
    << goal.x() << ", "
    << goal.y() << ", "
    << goal.z() << "]"
  << endl;

  // Run the algorithm
  MatrixXd jointVariables(4, 1);
  jointVariables << BIAS_BASE, BIAS_SHOULDER, BIAS_ELBOW, BIAS_WRIST;

  for (int iteration = 0; iteration < MAX_ITERATIONS; iteration++)
  {
    for (int joint = 0; joint < jointVariables.rows(); joint++)
    {
      double error = sample(goal, jointVariables, joint, DELTA);
      jointVariables(joint, 0) -= DAMPING * error;
    }

    double distance = distanceToGoal(goal, jointVariables);

    if (verbose)
    {
      cout << jointVariables << endl;
      cout << "distance " << distance << endl;
    }

    if (distance < TOLERANCE) break;
  }

  cout << endl << "Joints" << endl << jointVariables << endl;

  return 0;
}

double distanceToGoal(Vector3d goalPosition, MatrixXd jointVariables)
{
  Matrix4d endEffector = forwardKinematics(jointVariables);
  Vector3d position = endEffector.block<3, 1>(0, 3);
  return (goalPosition - position).norm();
}

double sample(Vector3d goal, MatrixXd jointVariables, int joint, double delta)
{
  double first = distanceToGoal(goal, jointVariables);

  MatrixXd secondVariables = jointVariables;
  secondVariables(joint, 0) += delta;

  double second = distanceToGoal(goal, secondVariables);

  return (second - first) / delta;
}
