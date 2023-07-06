#include <iostream>
#include <Eigen/Dense>
#include "fk.h"

using namespace std;
using namespace Eigen;

int main(int argc, char **argv)
{
  const double DRUMSTICK_OFFSET = -0.023;
  const double DRUMSTICK_LENGTH = 0.305;
  const double DRUMSTICK_ANGLE = 0.959931;
  const double FOREARM_LENGTH = 0.48059;
  const double UPPERARM_LENGTH = 0.4173;
  const double SHOULDER_OFFSET_X = -0.013;
  const double SHOULDER_OFFSET_Z = 0.11518;
  const double ORIGIN_X = 0.0;
  const double ORIGIN_Y = 0.0;
  const double ORIGIN_Z = 0.0;

  // Goal
  if (argc < 4)
  {
    cout << "Usage: " << argv[0] << " <x> <y> <z>" << endl;
    return 1;
  }

  double goalX = stod(argv[1]);
  double goalY = stod(argv[2]);
  double goalZ = stod(argv[3]);

  cout << "Inverse kinematics"
       << endl
       << endl;
  cout << "Given Position" << endl
       << "[" << endl
       << goalX << endl
       << goalY << endl
       << goalZ << endl
       << "]" << endl
       << endl;

  // Base
  double angleToGoal = atan2(goalY - ORIGIN_Y, goalX - ORIGIN_X);
  double distanceToGoalXY = sqrt(pow(goalX - ORIGIN_X, 2) + pow(goalY - ORIGIN_Y, 2));
  double angleWristOffset = abs(asin(DRUMSTICK_OFFSET / distanceToGoalXY));
  double baseAngle = angleToGoal - angleWristOffset;

  // Shoulder
  double shoulderX = SHOULDER_OFFSET_X;
  double shoulderZ = SHOULDER_OFFSET_Z;
  double shoulderToGoalDistance = sqrt(pow(distanceToGoalXY - shoulderX, 2) + pow(goalZ - shoulderZ, 2));
  double drumstickOffsetZ = sin(DRUMSTICK_ANGLE) * DRUMSTICK_LENGTH;
  double drumstickOffsetX = cos(DRUMSTICK_ANGLE) * DRUMSTICK_LENGTH;
  double elbowToGoalAngle = atan2(drumstickOffsetZ, FOREARM_LENGTH + drumstickOffsetX);
  double elbowToGoalDistance = sqrt(pow(FOREARM_LENGTH + drumstickOffsetX, 2) + pow(drumstickOffsetZ, 2));
  double innerShoulderAngle = acos(
    (pow(elbowToGoalDistance, 2) - pow(shoulderToGoalDistance, 2) - pow(UPPERARM_LENGTH, 2))
    /
    (-2 * shoulderToGoalDistance * UPPERARM_LENGTH)
  );
  double shoulderToGoalZ = goalZ - shoulderZ;
  double shoulderToGoalX = distanceToGoalXY - shoulderX;
  double shoulderToGoalAngle = atan2(shoulderToGoalZ, shoulderToGoalX);
  double shoulderAngle = shoulderToGoalAngle + innerShoulderAngle;

  // Elbow
  double outerElbowAngle = acos(
    (pow(shoulderToGoalDistance, 2) - pow(elbowToGoalDistance, 2) - pow(UPPERARM_LENGTH, 2))
    /
    (-2 * elbowToGoalDistance * UPPERARM_LENGTH)
  );
  double innerElbowAngle = outerElbowAngle - elbowToGoalAngle;
  double elbowAngle = -(M_PI - innerElbowAngle);

  // End Effector
  MatrixXd angles(3, 1);
  angles << baseAngle, shoulderAngle, elbowAngle;
  Matrix4d endEffector = forwardKinematics(str1ker, angles);

  // Output
  cout << "Base" << endl
       << baseAngle << endl << endl
       << "Shoulder" << endl
       << shoulderAngle << endl << endl
       << "Elbow" << endl
       << elbowAngle << endl << endl
       << "End Effector" << endl
       << endEffector << endl << endl;

  return 0;
}
