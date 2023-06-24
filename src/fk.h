#if __INTELLISENSE__
#undef __ARM_NEON
#undef __ARM_NEON__
#endif

#include <Eigen/Dense>

using namespace Eigen;

struct DHParams
{
  double a;
  double d;
  double alpha;
  double theta;
};

Matrix4d dh(DHParams parameters)
{
  return (
    Translation3d(0, 0, parameters.d) *
    AngleAxisd(parameters.theta, Vector3d::UnitZ()) *
    Translation3d(parameters.a, 0, 0) *
    AngleAxisd(parameters.alpha, Vector3d::UnitX())
  ).matrix();
}

Matrix4d str1kerBasic(int joint, double angle)
{
  switch (joint)
  {
    case 0: // base
      return (
        Translation3d(0, 0, 0) *
        AngleAxisd(M_PI / 2, Vector3d::UnitX()) *
        AngleAxisd(angle, Vector3d::UnitZ())
      ).matrix();
    case 1: // shoulder
      return (
        Translation3d(-0.013, 0.11518, 0) *
        AngleAxisd(angle, Vector3d::UnitZ())
      ).matrix();
    case 2: // elbow
      return (
        Translation3d(0.4173, 0, 0) *
        AngleAxisd(angle, Vector3d::UnitZ()) *
        Translation3d(0.48059, 0, -0.023)
      ).matrix();
    default:
      return Matrix4d::Identity();
  }
}

Matrix4d str1ker(int joint, double angle)
{
  switch (joint)
  {
    case 0: // base
      return dh((DHParams){ .a = 0,       .d = 0.11518, .alpha = M_PI / 2, .theta = angle }) *
             dh((DHParams){ .a = -0.013,  .d = 0,       .alpha = 0,        .theta = 0 });
    case 1: // shoulder
      return dh((DHParams){ .a = 0.4173,  .d = 0,       .alpha = 0,        .theta = angle });
    case 2: // elbow
      return dh((DHParams){ .a = 0.48059, .d = 0,       .alpha = 0,        .theta = angle }) *
             dh((DHParams){ .a = 0.008,   .d = 0,       .alpha = 0,        .theta = M_PI / 4 }) *
             dh((DHParams){ .a = 0.295,   .d = -0.023,  .alpha = 0,        .theta = -M_PI / 4 + 0.959931 });
    default:
      return Matrix4d::Identity();
  }
}

// this is how we start without DH parameters!

Matrix4d kuka(int joint, double angle)
{
  switch (joint)
  {
    case 0: // shoulder
      return (
        Translation3d(0, 0, 0.203) *
        AngleAxisd(angle, Vector3d::UnitZ())
      ).matrix();
    case 1: // bicep
      return (
        Translation3d(0.075, 0.0735, 0.13) *
        AngleAxisd(angle, Vector3d::UnitY())
      ).matrix();
    case 2: // elbow
      return (
        Translation3d(0, -0.0055, 0.27) *
        AngleAxisd(angle, Vector3d::UnitY())
      ).matrix();
    case 3: // wrist
      return (
        Translation3d(0.106, -0.068001, 0.09) *
        AngleAxisd(angle, Vector3d::UnitX())
      ).matrix();
    case 4: // hand
      return (
        Translation3d(0.187, -0.029, 0) *
        AngleAxisd(angle, Vector3d::UnitY())
      ).matrix();
    case 5: // effector
      return (
        Translation3d(0.052, 0.029, 0) *
        AngleAxisd(angle, Vector3d::UnitX())
      ).matrix();
    default:
      return Matrix4d::Identity();
  }
}

Matrix4d forwardKinematics(Matrix4d (*description)(int, double), MatrixXd angles)
{
  Matrix4d endEffector = Matrix4d::Identity();

  for (int n = 0; n < angles.rows(); n++)
  {
    endEffector = endEffector * description(n, angles(n, 0));
  }

  return endEffector;
}
