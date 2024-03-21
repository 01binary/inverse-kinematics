#include <Eigen/Dense>

using namespace Eigen;

Matrix4d forwardKinematics(MatrixXd jointVariables)
{
  return
    // Base
    (
      Translation3d(0, 0, 0) *
      AngleAxisd(jointVariables(0,0), Vector3d::UnitZ())
    ).matrix() *
    // Shoulder
    (
      Translation3d(0, 0, 0.670) *
      AngleAxisd(jointVariables(1,0), Vector3d::UnitY())
    ).matrix() *
    // Elbow
    (
      Translation3d(0.7, 0, 0) *
      AngleAxisd(jointVariables(2,0), Vector3d::UnitY())
    ).matrix() *
    // Wrist
    (
      Translation3d(0.7, 0.05, 0) *
      AngleAxisd(jointVariables(3,0), Vector3d::UnitX())
    ).matrix() *
    (
      Translation3d(0.18, 0, 0) *
      AngleAxisd(0.0, Vector3d::UnitX())
    ).matrix();
}
