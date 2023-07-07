#include <iostream>
#include <Eigen/Dense>
#include "fk.h"

using namespace std;
using namespace Eigen;

Vector3d forwardKinematicsPose(MatrixXd angles)
{
  Matrix4d endEffector = forwardKinematics(str1ker, angles);
  return endEffector.block<3, 1>(0, 3);
}

MatrixXd calculateJacobian(const VectorXd& angles)
{
  const double EPSILON = 1e-6;
  const int joints = angles.size();

  Vector3d pose = forwardKinematicsPose(angles);
  MatrixXd jacobian(3, joints);

  for (int n = 0; n < joints; ++n)
  {
    VectorXd deltaAngles = VectorXd::Zero(joints);
    deltaAngles(n) = EPSILON;

    const Vector3d diff = forwardKinematicsPose(angles + deltaAngles) - pose;

    jacobian.block<3, 1>(0, n) = diff / EPSILON;
  }

  return jacobian;
}

int main(int argc, char** argv)
{
  const int MAX_ITERATIONS = 10000;
  const double TOLERANCE = 0.001;
  const double DAMPING = 0.5;

  const double BASE_BIAS = 0;
  const double SHOULDER_BIAS = 1;
  const double ELBOW_BIAS = -1;

  if (argc < 4)
  {
    cout << "Usage: " << argv[0] << " <x> <y> <z>" << endl;
    return 1;
  }

  cout << "Inverse kinematics" << endl << endl;
  cout << "Given Position" << endl;
  cout << "[ "
    << argv[1] << ", "
    << argv[2] << ", "
    << argv[3]
    << " ]"
    << endl
    << endl;

  bool verbose = argc > 4 && string(argv[4]) == "--verbose";

  Vector3d target;
  target << stod(argv[1]), stod(argv[2]), stod(argv[3]);

  VectorXd angles(3);
  angles << BASE_BIAS, SHOULDER_BIAS, ELBOW_BIAS;
  
  for (int iteration = 0; iteration < MAX_ITERATIONS; iteration++)
  {
    Vector3d pose = forwardKinematicsPose(angles);
    Vector3d error = target - pose;

    if (error.norm() < TOLERANCE) break;

    MatrixXd Jtranspose = calculateJacobian(angles).transpose();
    angles = angles + DAMPING * (Jtranspose * error);
  }

  cout << endl << "Angles" << endl << angles << endl;

  return 0;
}
