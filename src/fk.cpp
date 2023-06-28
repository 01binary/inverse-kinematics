#include <iostream>
#include "fk.h"

using namespace std;

int main(int argc, char** argv)
{
  if (argc < 2)
  {
    cout << "Usage: " << argv[0] << " <angle1> <angle2> <anglen>" << endl;
    return 1;
  }

  MatrixXd angles(argc - 1, 1);

  for (int n = 1; n < argc; n++)
  {
    angles(n - 1, 0) = atof(argv[n]);
  }

  Matrix4d pose = forwardKinematics(str1kerBasic, angles);

  cout << "Forward Kinematics" << endl << endl;
  cout << "Given Angles" << endl;
  cout << angles << endl << endl;
  cout << "End Effector" << endl << endl;
  cout << pose << endl;

  return 0;
}
