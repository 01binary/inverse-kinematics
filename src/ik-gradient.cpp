#include <iostream>
#include <Eigen/Dense>
#include "fk.h"

using namespace std;
using namespace Eigen;

double distanceToTarget(Vector3d target, MatrixXd angles)
{
  Matrix4d endEffector = forwardKinematics(str1ker, angles);
  Vector3d pose = endEffector.block<3, 1>(0, 3);
  return (target - pose).norm();
}

double sample(Vector3d target, MatrixXd angles, int joint, double samplingDistance)
{
  double first = distanceToTarget(target, angles);

  MatrixXd secondAngles = angles;
  secondAngles(joint, 0) += samplingDistance;

  double second = distanceToTarget(target, secondAngles);

  return (second - first) / samplingDistance;
}

int main(int argc, char** argv)
{
  const int MAX_ITERATIONS = 10000;
  const double SAMPLING_DISTANCE = 0.001;
  const double DAMPING = 0.001;
  const double TOLERANCE = 0.0001;

  const double BIAS_BASE = 0;
  const double BIAS_SHOULDER = 1;
  const double BIAS_ELBOW = -1;

  if (argc < 4)
  {
    cout << "Usage: " << argv[0] << " <x> <y> <z>" << endl;
    return 1;
  }

  Vector3d target;
  target << stod(argv[1]), stod(argv[2]), stod(argv[3]);

  bool verbose = argc > 4 && string(argv[4]) == "--verbose";

  cout << "Inverse kinematics" << endl << endl;
  cout << "Given Position" << endl
    << "[" << endl
    << target << endl
    << "]" << endl
    << endl;

  MatrixXd angles(3, 1);
  angles << BIAS_BASE, BIAS_SHOULDER, BIAS_ELBOW;

  double distance;

  for (int iteration = 0; iteration < MAX_ITERATIONS; iteration++)
  {
    for (int joint = 0; joint < angles.rows(); joint++)
    {
      double delta = sample(target, angles, joint, SAMPLING_DISTANCE);
      angles(joint, 0) -= DAMPING * delta;
    }

    double distance = distanceToTarget(target, angles);

    if (verbose)
    {
      cout << angles << endl;
      cout << "distance " << distance << endl;
    }

    if (distance < TOLERANCE) break;
  }

  cout << endl << "Angles" << endl << angles << endl;

  return 0;
}
