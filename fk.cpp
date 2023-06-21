#include <iostream>
#include "fk.h"

using namespace std;

int main(int argc, char** argv)
{
  if (argc < 4)
  {
    cout << "Usage: " << argv[0] << " <base> <shoulder> <elbow>" << endl;
    return 1;
  }

  MatrixXd angles(3, 1);
  angles << atof(argv[1]), atof(argv[2]), atof(argv[3]);

  Matrix4d pose = forwardKinematics(angles);

  cout << "Forward Kinematics" << endl << endl;
  cout << "Given Angles" << endl;
  cout << angles << endl << endl;
  cout << "End Effector" << endl << endl;
  cout << pose << endl;

  return 0;
}
