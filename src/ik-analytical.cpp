#include <iostream>
#include <vector>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

inline double pow2(double x)
{
  return x * x;
}

int main(int argc, char **argv)
{
  // Goal
  cout << "Inverse kinematics"
       << endl
       << endl;
  cout << "Enter Position and Orientation Matrix" << endl
       << "[" << endl;

  vector<vector<double>> goal;

  for (int n = 0; n < 3; ++n)
  {
    cout << "  ";

    string line;
    getline(cin, line);

    vector<double> row;
    stringstream stream(line);

    while (stream.good())
    {
      string cell;
      getline(stream, cell, ',');
      row.push_back(stod(cell));
    }

    goal.push_back(row);
  }

  goal.push_back({0, 0, 0, 1});

  cout << "  0, 0, 0, 1" << endl;
  cout << "]" << endl << endl;

  double nx = goal[0][0];
  double ny = goal[1][0];
  double nz = goal[2][0];

  double ox = goal[0][1];
  double oy = goal[1][1];
  double oz = goal[2][1];

  double ax = goal[0][2];
  double ay = goal[1][2];
  double az = goal[2][2];

  double px = goal[0][3];
  double py = goal[1][3];
  double pz = goal[2][3];

  // Constants
  const double SQRT2 = sqrt(2.0);

  // Base angle
  double baseAngle = asin(ax);

  // Shoulder angle
  double shoulderAngle = acos(
    (
      (230.0 * ax) / 4173.0 +
      (10000.0 * px) / 4173.0 +
      (10.0 * cos(baseAngle)) / 321.0 -
      (
        nx * (
          1600.0 * cos(1572052035774335.0 / 9007199254740992.0) +
          48059.0 * SQRT2 * cos(1572052035774335.0 / 9007199254740992.0) -
          48059.0 * SQRT2 * sin(1572052035774335.0 / 9007199254740992.0) +
          61000.0 * pow2(cos(1572052035774335.0 / 9007199254740992.0)) +
          61000.0 * pow2(sin(1572052035774335.0 / 9007199254740992.0))
        )
      )
      /
      (
        83460.0 * (
          pow2(cos(1572052035774335.0 / 9007199254740992.0)) +
          pow2(sin(1572052035774335.0 / 9007199254740992.0))
        )
      )
      + (
        ox * (
          48059.0 * cos(1572052035774335.0 / 9007199254740992.0) +
          48059.0 * sin(1572052035774335.0 / 9007199254740992.0) +
          800.0 * SQRT2 * sin(1572052035774335.0 / 9007199254740992.0)
        )
      )
      / (
        41730.0 * (
          SQRT2 * pow2(cos(1572052035774335.0 / 9007199254740992.0)) +
          SQRT2 * pow2(sin(1572052035774335.0 / 9007199254740992.0))
        )
      )
    )
    / cos(baseAngle)
  );

  // Elbow angle
  double elbowAngle = M_PI - acos((SQRT2*(nx*oz - nz*ox)*(ox*cos(1572052035774335/9007199254740992)*cos(shoulderAngle) - nx*cos(1572052035774335/9007199254740992)*cos(shoulderAngle) + nx*sin(1572052035774335/9007199254740992)*cos(shoulderAngle) + ox*sin(1572052035774335/9007199254740992)*cos(shoulderAngle) - nz*cos(1572052035774335/9007199254740992)*cos(baseAngle)*sin(shoulderAngle) + oz*cos(1572052035774335/9007199254740992)*cos(baseAngle)*sin(shoulderAngle) + nz*sin(1572052035774335/9007199254740992)*cos(baseAngle)*sin(shoulderAngle) + oz*sin(1572052035774335/9007199254740992)*cos(baseAngle)*sin(shoulderAngle)))/(2*(pow2(nx)*pow2(cos(shoulderAngle)) + pow2(ox)*pow2(cos(shoulderAngle)) + pow2(nz)*pow2(cos(baseAngle))*pow2(sin(shoulderAngle)) + pow2(oz)*pow2(cos(baseAngle))*pow2(sin(shoulderAngle)) + 2*nx*nz*cos(baseAngle)*cos(shoulderAngle)*sin(shoulderAngle) + 2*ox*oz*cos(baseAngle)*cos(shoulderAngle)*sin(shoulderAngle))));

  // Output
  cout
    << endl
    << "Base" << endl
    << baseAngle << endl << endl
    << "Shoulder" << endl
    << shoulderAngle << endl << endl
    << "Elbow" << endl
    << elbowAngle << endl << endl;

  return 0;
}
