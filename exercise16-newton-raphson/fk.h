#include <Eigen/Dense>

using namespace Eigen;

enum JOINTS
{
  BASE,
  SHOULDER,
  ELBOW,
  WRIST
};

Matrix4d forwardKinematics(MatrixXd jointVariables)
{
  Matrix4d endEffector = Matrix4d::Identity();

  for (int n = 0; n < jointVariables.rows(); n++)
  {
    switch (n)
    {
      case BASE:
        endEffector = endEffector * (
          Translation3d(0, 0, 0) *
          AngleAxisd(jointVariables(n, 0), Vector3d::UnitZ())
        ).matrix();
        break;
      case SHOULDER:
        endEffector = endEffector * (
          Translation3d(0, 0, 0.670) *
          AngleAxisd(jointVariables(n, 0), Vector3d::UnitY())
        ).matrix();
        break;
      case ELBOW:
        endEffector = endEffector * (
          Translation3d(0.7, 0, 0) *
          AngleAxisd(jointVariables(n, 0), Vector3d::UnitY())
        ).matrix();
        break;
      case WRIST:
        endEffector = endEffector * (
          Translation3d(0.7, 0.05, 0) *
          AngleAxisd(jointVariables(n, 0), Vector3d::UnitX())
        ).matrix();
        break;
    }
  }
}
