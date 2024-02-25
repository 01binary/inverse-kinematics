#include <iostream>
#include "fk.h"

using namespace std;

Vector3d sample(MatrixXd jointVariables);
MatrixXd calculateJacobian(const VectorXd& jointVariables);

int main(int argc, char** argv)
{
  // Algorithm parameters
  const int MAX_ITERATIONS = 10000;
  const double TOLERANCE = 0.001;
  const double DAMPING = 0.5;

  // Initial joint states
  const double BASE_BIAS = 0;
  const double SHOULDER_BIAS = -1;
  const double ELBOW_BIAS = 1;
  const double WRIST_BIAS = 0;

  // Print usage if not enough arguments
  if (argc < 4)
  {
    cout << "Usage: " << argv[0] << " <x> <y> <z>" << endl;
    return 1;
  }

  bool verbose = argc > 4 && string(argv[4]) == "--verbose";

  // Prompt for goal position
  Vector3d goal;
  goal << stod(argv[1]), stod(argv[2]), stod(argv[3]);

  cout << "Inverse Kinematics (Newton-Raphson Iterator)" << endl << endl;
  cout << "Given Pose ["
    << goal.x() << ", "
    << goal.y() << ", "
    << goal.z() << "]"
  << endl;

  // Run the algorithm
  VectorXd jointVariables(4);
  jointVariables << BASE_BIAS, SHOULDER_BIAS, ELBOW_BIAS, WRIST_BIAS;
  
  for (int iteration = 0; iteration < MAX_ITERATIONS; iteration++)
  {
    Vector3d pose = sample(jointVariables);
    Vector3d error = goal - pose;

    if (error.norm() < TOLERANCE) break;

    MatrixXd Jtranspose = calculateJacobian(jointVariables).transpose();
    jointVariables = jointVariables + DAMPING * (Jtranspose * error);
  }

  cout << endl << "Joints" << endl << jointVariables << endl;

  return 0;
}

Vector3d sample(MatrixXd jointVariables)
{
  Matrix4d endEffector = forwardKinematics(jointVariables);
  return endEffector.block<3, 1>(0, 3);
}

MatrixXd calculateJacobian(const VectorXd& jointVariables)
{
  const double EPSILON = 1e-6;
  const int joints = jointVariables.size();

  Vector3d pose = sample(jointVariables);
  MatrixXd jacobian(3, joints);

  for (int n = 0; n < joints; ++n)
  {
    VectorXd deltaAngles = VectorXd::Zero(joints);
    deltaAngles(n) = EPSILON;

    const Vector3d diff = sample(jointVariables + deltaAngles) - pose;

    jacobian.block<3, 1>(0, n) = diff / EPSILON;
  }

  return jacobian;
}
