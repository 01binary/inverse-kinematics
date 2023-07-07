#include <iostream>
#include <vector>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

vector<vector<double>> inputGoal()
{
  cout << "Enter Position and Orientation Matrix"
       << endl << "[" << endl;

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

  return goal;
}

int main(int argc, char **argv)
{
  cout << "Inverse kinematics" << endl << endl;

  // Goal
  auto goal = inputGoal();

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

  // Base angle
  double baseSin = ax;
  double baseCos = -ay;
  double base = atan2(baseSin, baseCos);

  // Shoulder angle
  double shoulderSin =
    az * 5.511622334052241E-2 -
    nz * 1.410336919654 + oz * 9.467180476060177E-1 +
    pz * 2.396357536544452 - 2.7601246105919E-1;

  double shoulderCos = (
    (
      ax * 2.3E-2 -
      ay * 1.3E-2 -
      nx * 5.88533596571614E-1 +
      ox * 3.950654412659912E-1 +
      px
    ) * -2.396357536544452
  ) / ay;

  double shoulder = atan2(shoulderSin, shoulderCos);
  
  // Elbow angle
  double elbow = -acos(
    (
      (
        nx * oz - nz * ox
      )
      *
      (
        nx * shoulderCos * 8.111596779808571E-1 -
        ox * shoulderCos* 1.158455858812925 +
        nz * cos(base) * shoulder * 8.111596779808571E-1 -
        oz * cos(base) * shoulder * 1.158455858812925
      ) * 7.071067811865475E-1
    )
    /
    (
      (nx * nx) * pow(shoulderCos, 2.0) +
      (ox * ox) * pow(shoulderCos, 2.0) +
      (nz * nz) * pow(cos(base), 2.0) * pow(shoulder, 2.0) +
      (oz * oz) * pow(cos(base), 2.0) * pow(shoulder, 2.0) +
      nx * nz * cos(base) * shoulderCos * shoulder * 2.0 +
      ox * oz * cos(base) * shoulderCos * shoulder * 2.0
    )
  );

  // Output
  cout
    << endl
    << "Base" << endl
    << base << endl << endl
    << "Shoulder" << endl
    << shoulder << endl << endl
    << "Elbow" << endl
    << elbow << endl << endl;

  return 0;
}
