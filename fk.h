#include <Eigen/Dense>

using namespace Eigen;

enum JOINTS
{
  BASE,
  SHOULDER,
  ELBOW
};

Matrix4d denavitHartenberg(double a, double d, double alpha, double theta)
{
  return (
    Translation3d(0, 0, d) *
    AngleAxisd(theta, Vector3d::UnitZ()) *
    Translation3d(a, 0, 0) *
    AngleAxisd(alpha, Vector3d::UnitX())
  ).matrix();
}

Matrix4d jointFrame(int index, double angle)
{
  switch (index)
  {
    case BASE:
      return denavitHartenberg(0, 0.11518, M_PI / 2, angle) *
             denavitHartenberg(-0.013, 0, 0, 0);
    case SHOULDER:
      return denavitHartenberg(0.4173, 0, 0, angle);
    case ELBOW:
      return denavitHartenberg(0.48059, 0, 0, angle) *
             denavitHartenberg(0.008, 0, 0, M_PI / 4) *
             denavitHartenberg(0.295, -0.023, 0, -M_PI / 4 + 0.959931);
    default:
      return Matrix4d::Identity();
  }
}

Matrix4d forwardKinematics(MatrixXd angles)
{
  Matrix4d endEffector = Matrix4d::Identity();

  for (int n = 0; n < angles.rows(); n++)
  {
    endEffector = endEffector * jointFrame(n, angles(n, 0));
  }

  return endEffector;
}
