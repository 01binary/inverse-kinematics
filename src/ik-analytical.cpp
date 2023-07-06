#include <iostream>
#include <vector>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

const double SQRT2 = sqrt(2.0);

inline double pow2(double x)
{
  return x * x;
}

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
  double base = atan2(ax, -ay);

  // Shoulder angle
  double shoulderSin =
    az * 5.511622334052241E-2 -
    nz * 1.410336919654+oz*9.467180476060177E-1 +
    pz * 2.396357536544452-2.7601246105919E-1;

  double shoulderCos = (
    (
      ax * 2.3E-2-ay*1.3E-2-
      nx * 5.88533596571614E-1+
      ox * 3.950654412659912E-1+px
    ) * -2.396357536544452
  ) / ay;

  double shoulder = atan2(shoulderSin, shoulderCos);
  
  // Elbow angle
  double elbowCos = ((nx*oz-nz*ox*1.0)*(nx*cos(shoulder)*8.111596779808571E-1-ox*cos(shoulder)*1.158455858812925+nz*cos(base)*sin(shoulder)*8.111596779808571E-1-oz*cos(base)*sin(shoulder)*1.158455858812925)*7.071067811865475E-1)/((nx*nx)*pow(cos(shoulder),2.0)+(ox*ox)*pow(cos(shoulder),2.0)+(nz*nz)*pow(cos(base),2.0)*pow(sin(shoulder),2.0)+(oz*oz)*pow(cos(base),2.0)*pow(sin(shoulder),2.0)+nx*nz*cos(base)*cos(shoulder)*sin(shoulder)*2.0+ox*oz*cos(base)*cos(shoulder)*sin(shoulder)*2.0);
  double elbowSin = ((sqrt(((nz*nz*nz*nz)*pow(cos(base),5.0)*pow(sin(shoulder),5.0)*1.131370849898476E+65+(oz*oz*oz*oz)*pow(cos(base),5.0)*pow(sin(shoulder),5.0)*1.131370849898476E+65+(nx*nx*nx*nx)*cos(base)*pow(cos(shoulder),4.0)*sin(shoulder)*1.131370849898476E+65+(ox*ox*ox*ox)*cos(base)*pow(cos(shoulder),4.0)*sin(shoulder)*1.131370849898476E+65+(nz*nz)*(oz*oz)*pow(cos(base),5.0)*pow(sin(shoulder),5.0)*2.262741699796952E+65+(nx*nx*nx)*nz*pow(cos(base),2.0)*pow(cos(shoulder),3.0)*pow(sin(shoulder),2.0)*4.525483399593904E+65+(ox*ox*ox)*oz*pow(cos(base),2.0)*pow(cos(shoulder),3.0)*pow(sin(shoulder),2.0)*4.525483399593904E+65+(nx*nx)*(nz*nz)*pow(cos(base),3.0)*pow(cos(shoulder),2.0)*pow(sin(shoulder),3.0)*6.788225099390856E+65+(nx*nx)*(oz*oz)*pow(cos(base),3.0)*pow(cos(shoulder),2.0)*pow(sin(shoulder),3.0)*2.262741699796952E+65+(nz*nz)*(ox*ox)*pow(cos(base),3.0)*pow(cos(shoulder),2.0)*pow(sin(shoulder),3.0)*2.262741699796952E+65+(ox*ox)*(oz*oz)*pow(cos(base),3.0)*pow(cos(shoulder),2.0)*pow(sin(shoulder),3.0)*6.788225099390856E+65+nx*(nz*nz*nz)*pow(cos(base),4.0)*cos(shoulder)*pow(sin(shoulder),4.0)*4.525483399593904E+65+(nx*nx)*(ox*ox)*cos(base)*pow(cos(shoulder),4.0)*sin(shoulder)*2.262741699796952E+65+ox*(oz*oz*oz)*pow(cos(base),4.0)*cos(shoulder)*pow(sin(shoulder),4.0)*4.525483399593904E+65+nx*nz*(oz*oz)*pow(cos(base),4.0)*cos(shoulder)*pow(sin(shoulder),4.0)*4.525483399593904E+65+(nz*nz)*ox*oz*pow(cos(base),4.0)*cos(shoulder)*pow(sin(shoulder),4.0)*4.525483399593904E+65+nx*nz*(ox*ox)*pow(cos(base),2.0)*pow(cos(shoulder),3.0)*pow(sin(shoulder),2.0)*4.525483399593904E+65+(nx*nx)*ox*oz*pow(cos(base),2.0)*pow(cos(shoulder),3.0)*pow(sin(shoulder),2.0)*4.525483399593904E+65+nx*nz*ox*oz*pow(cos(base),3.0)*pow(cos(shoulder),2.0)*pow(sin(shoulder),3.0)*9.050966799187808E+65)*(nz*(ox*ox*ox*ox*ox)*pow(cos(shoulder),3.0)*5.315704537936382E+64+(nx*nx*nx*nx*nx)*oz*pow(cos(shoulder),3.0)*5.315704537936382E+64+nx*nz*(ox*ox*ox*ox)*pow(cos(shoulder),3.0)*3.86951431854829E+64-(nx*nx*nx*nx)*nz*ox*pow(cos(shoulder),3.0)*5.315704537936382E+64-nx*(ox*ox*ox*ox)*oz*pow(cos(shoulder),3.0)*5.315704537936382E+64-(nx*nx*nx*nx)*ox*oz*pow(cos(shoulder),3.0)*3.86951431854829E+64+(nx*nx)*(oz*oz*oz*oz)*pow(cos(base),3.0)*pow(sin(shoulder),3.0)*1.949262320140071E+31+(nz*nz*nz*nz)*(ox*ox)*pow(cos(base),3.0)*pow(sin(shoulder),3.0)*5.08715984893836E+30+(nx*nx)*nz*(ox*ox*ox)*pow(cos(shoulder),3.0)*6.383665090629151E+30+(nx*nx*nx)*nz*(ox*ox)*pow(cos(shoulder),3.0)*3.86951431854829E+64-(nx*nx)*(ox*ox*ox)*oz*pow(cos(shoulder),3.0)*3.86951431854829E+64-(nx*nx*nx)*(ox*ox)*oz*pow(cos(shoulder),3.0)*6.383665090629151E+30-(nx*nx)*(nz*nz)*(oz*oz)*pow(cos(base),3.0)*pow(sin(shoulder),3.0)*3.86951431854829E+64+(nz*nz)*(ox*ox)*(oz*oz)*pow(cos(base),3.0)*pow(sin(shoulder),3.0)*3.86951431854829E+64-(nx*nx*nx)*(oz*oz*oz)*pow(cos(base),2.0)*cos(shoulder)*pow(sin(shoulder),2.0)*5.315704537936382E+64-(nz*nz*nz)*(ox*ox*ox)*pow(cos(base),2.0)*cos(shoulder)*pow(sin(shoulder),2.0)*5.315704537936382E+64-nx*(nz*nz*nz*nz)*ox*pow(cos(base),3.0)*pow(sin(shoulder),3.0)*5.315704537936382E+64-nx*ox*(oz*oz*oz*oz)*pow(cos(base),3.0)*pow(sin(shoulder),3.0)*5.315704537936382E+64-(nx*nx)*nz*(oz*oz*oz)*pow(cos(base),3.0)*pow(sin(shoulder),3.0)*5.315704537936382E+64+(nx*nx)*(nz*nz*nz)*oz*pow(cos(base),3.0)*pow(sin(shoulder),3.0)*5.315704537936382E+64+nz*(ox*ox)*(oz*oz*oz)*pow(cos(base),3.0)*pow(sin(shoulder),3.0)*5.315704537936382E+64-(nz*nz*nz)*(ox*ox)*oz*pow(cos(base),3.0)*pow(sin(shoulder),3.0)*5.315704537936382E+64-(nx*nx*nx*nx)*(oz*oz)*cos(base)*pow(cos(shoulder),2.0)*sin(shoulder)*3.86951431854829E+64+(nz*nz)*(ox*ox*ox*ox)*cos(base)*pow(cos(shoulder),2.0)*sin(shoulder)*3.86951431854829E+64+(nx*nx*nx*nx)*nz*oz*cos(base)*pow(cos(shoulder),2.0)*sin(shoulder)*1.594711361380914E+65+nz*(ox*ox*ox*ox)*oz*cos(base)*pow(cos(shoulder),2.0)*sin(shoulder)*1.594711361380914E+65+nx*(nz*nz)*ox*(oz*oz)*pow(cos(base),3.0)*pow(sin(shoulder),3.0)*1.063140907587276E+65-nx*(nz*nz)*(ox*ox*ox)*cos(base)*pow(cos(shoulder),2.0)*sin(shoulder)*5.315704537936382E+64-(nx*nx*nx)*(nz*nz)*ox*cos(base)*pow(cos(shoulder),2.0)*sin(shoulder)*1.594711361380914E+65-nx*(ox*ox*ox)*(oz*oz)*cos(base)*pow(cos(shoulder),2.0)*sin(shoulder)*1.594711361380914E+65-(nx*nx*nx)*ox*(oz*oz)*cos(base)*pow(cos(shoulder),2.0)*sin(shoulder)*5.315704537936382E+64+(nx*nx)*(nz*nz)*(ox*ox)*cos(base)*pow(cos(shoulder),2.0)*sin(shoulder)*7.73902863709658E+64+nx*(nz*nz*nz)*(ox*ox)*pow(cos(base),2.0)*cos(shoulder)*pow(sin(shoulder),2.0)*3.86951431854829E+64-(nx*nx)*(nz*nz*nz)*ox*pow(cos(base),2.0)*cos(shoulder)*pow(sin(shoulder),2.0)*1.594711361380914E+65-(nx*nx*nx)*nz*(oz*oz)*pow(cos(base),2.0)*cos(shoulder)*pow(sin(shoulder),2.0)*7.73902863709658E+64+(nx*nx*nx)*(nz*nz)*oz*pow(cos(base),2.0)*cos(shoulder)*pow(sin(shoulder),2.0)*1.594711361380914E+65-(nx*nx)*(ox*ox)*(oz*oz)*cos(base)*pow(cos(shoulder),2.0)*sin(shoulder)*7.73902863709658E+64-nx*(ox*ox)*(oz*oz*oz)*pow(cos(base),2.0)*cos(shoulder)*pow(sin(shoulder),2.0)*1.594711361380914E+65-(nx*nx)*ox*(oz*oz*oz)*pow(cos(base),2.0)*cos(shoulder)*pow(sin(shoulder),2.0)*3.86951431854829E+64+nz*(ox*ox*ox)*(oz*oz)*pow(cos(base),2.0)*cos(shoulder)*pow(sin(shoulder),2.0)*1.594711361380914E+65+(nz*nz)*(ox*ox*ox)*oz*pow(cos(base),2.0)*cos(shoulder)*pow(sin(shoulder),2.0)*7.73902863709658E+64-nx*nz*ox*(oz*oz*oz)*pow(cos(base),3.0)*pow(sin(shoulder),3.0)*3.86951431854829E+64+nx*(nz*nz*nz)*ox*oz*pow(cos(base),3.0)*pow(sin(shoulder),3.0)*3.86951431854829E+64-nx*nz*(ox*ox)*(oz*oz)*pow(cos(base),2.0)*cos(shoulder)*pow(sin(shoulder),2.0)*3.86951431854829E+64+nx*(nz*nz)*(ox*ox)*oz*pow(cos(base),2.0)*cos(shoulder)*pow(sin(shoulder),2.0)*5.315704537936382E+64+(nx*nx)*nz*ox*(oz*oz)*pow(cos(base),2.0)*cos(shoulder)*pow(sin(shoulder),2.0)*5.315704537936382E+64+(nx*nx)*(nz*nz)*ox*oz*pow(cos(base),2.0)*cos(shoulder)*pow(sin(shoulder),2.0)*3.86951431854829E+64+nx*nz*(ox*ox*ox)*oz*cos(base)*pow(cos(shoulder),2.0)*sin(shoulder)*3.86951431854829E+64-(nx*nx*nx)*nz*ox*oz*cos(base)*pow(cos(shoulder),2.0)*sin(shoulder)*3.86951431854829E+64+(nx*nx)*nz*(ox*ox)*oz*cos(base)*pow(cos(shoulder),2.0)*sin(shoulder)*1.063140907587276E+65)*-1.0+pow((nx*nx*nx*nx*nx)*pow(cos(shoulder),4.0)*6.489277423846856E+64-(ox*ox*ox*ox*ox)*pow(cos(shoulder),4.0)*9.267646870503402E+64-(nx*nx)*(ox*ox*ox)*pow(cos(shoulder),4.0)*1.85352937410068E+65+(nx*nx*nx)*(ox*ox)*pow(cos(shoulder),4.0)*1.297855484769371E+65+nx*(ox*ox*ox*ox)*pow(cos(shoulder),4.0)*6.489277423846856E+64-(nx*nx*nx*nx)*ox*pow(cos(shoulder),4.0)*9.267646870503402E+64+nx*(nz*nz*nz*nz)*pow(cos(base),4.0)*pow(sin(shoulder),4.0)*6.489277423846856E+64+nx*(oz*oz*oz*oz)*pow(cos(base),4.0)*pow(sin(shoulder),4.0)*6.489277423846856E+64-(nz*nz*nz*nz)*ox*pow(cos(base),4.0)*pow(sin(shoulder),4.0)*9.267646870503402E+64-ox*(oz*oz*oz*oz)*pow(cos(base),4.0)*pow(sin(shoulder),4.0)*9.267646870503402E+64+(nx*nx)*(nz*nz*nz)*pow(cos(base),3.0)*cos(shoulder)*pow(sin(shoulder),3.0)*2.595710969538743E+65-(ox*ox)*(oz*oz*oz)*pow(cos(base),3.0)*cos(shoulder)*pow(sin(shoulder),3.0)*3.707058748201361E+65+(nx*nx*nx*nx)*nz*cos(base)*pow(cos(shoulder),3.0)*sin(shoulder)*2.595710969538743E+65-(ox*ox*ox*ox)*oz*cos(base)*pow(cos(shoulder),3.0)*sin(shoulder)*3.707058748201361E+65+(nx*nx*nx)*(nz*nz)*pow(cos(base),2.0)*pow(cos(shoulder),2.0)*pow(sin(shoulder),2.0)*3.893566454308114E+65+(nx*nx*nx)*(oz*oz)*pow(cos(base),2.0)*pow(cos(shoulder),2.0)*pow(sin(shoulder),2.0)*1.297855484769371E+65-(nz*nz)*(ox*ox*ox)*pow(cos(base),2.0)*pow(cos(shoulder),2.0)*pow(sin(shoulder),2.0)*1.85352937410068E+65-(ox*ox*ox)*(oz*oz)*pow(cos(base),2.0)*pow(cos(shoulder),2.0)*pow(sin(shoulder),2.0)*5.560588122302041E+65+nx*(nz*nz)*(oz*oz)*pow(cos(base),4.0)*pow(sin(shoulder),4.0)*1.297855484769371E+65-(nz*nz)*ox*(oz*oz)*pow(cos(base),4.0)*pow(sin(shoulder),4.0)*1.85352937410068E+65-nx*nz*(ox*ox*ox)*cos(base)*pow(cos(shoulder),3.0)*sin(shoulder)*3.707058748201361E+65-(nx*nx*nx)*nz*ox*cos(base)*pow(cos(shoulder),3.0)*sin(shoulder)*3.707058748201361E+65+nx*(ox*ox*ox)*oz*cos(base)*pow(cos(shoulder),3.0)*sin(shoulder)*2.595710969538743E+65+(nx*nx*nx)*ox*oz*cos(base)*pow(cos(shoulder),3.0)*sin(shoulder)*2.595710969538743E+65+nx*(nz*nz)*(ox*ox)*pow(cos(base),2.0)*pow(cos(shoulder),2.0)*pow(sin(shoulder),2.0)*1.297855484769371E+65-(nx*nx)*(nz*nz)*ox*pow(cos(base),2.0)*pow(cos(shoulder),2.0)*pow(sin(shoulder),2.0)*5.560588122302041E+65+nx*(ox*ox)*(oz*oz)*pow(cos(base),2.0)*pow(cos(shoulder),2.0)*pow(sin(shoulder),2.0)*3.893566454308114E+65-(nx*nx)*ox*(oz*oz)*pow(cos(base),2.0)*pow(cos(shoulder),2.0)*pow(sin(shoulder),2.0)*1.85352937410068E+65+(nx*nx)*nz*(ox*ox)*cos(base)*pow(cos(shoulder),3.0)*sin(shoulder)*2.595710969538743E+65-nx*(nz*nz*nz)*ox*pow(cos(base),3.0)*cos(shoulder)*pow(sin(shoulder),3.0)*3.707058748201361E+65-(nx*nx)*(ox*ox)*oz*cos(base)*pow(cos(shoulder),3.0)*sin(shoulder)*3.707058748201361E+65+nx*ox*(oz*oz*oz)*pow(cos(base),3.0)*cos(shoulder)*pow(sin(shoulder),3.0)*2.595710969538743E+65+(nx*nx)*nz*(oz*oz)*pow(cos(base),3.0)*cos(shoulder)*pow(sin(shoulder),3.0)*2.595710969538743E+65-(nz*nz)*(ox*ox)*oz*pow(cos(base),3.0)*cos(shoulder)*pow(sin(shoulder),3.0)*3.707058748201361E+65-nx*nz*(ox*ox)*oz*pow(cos(base),2.0)*pow(cos(shoulder),2.0)*pow(sin(shoulder),2.0)*7.414117496402722E+65+(nx*nx)*nz*ox*oz*pow(cos(base),2.0)*pow(cos(shoulder),2.0)*pow(sin(shoulder),2.0)*5.191421939077485E+65-nx*nz*ox*(oz*oz)*pow(cos(base),3.0)*cos(shoulder)*pow(sin(shoulder),3.0)*3.707058748201361E+65+nx*(nz*nz)*ox*oz*pow(cos(base),3.0)*cos(shoulder)*pow(sin(shoulder),3.0)*2.595710969538743E+65,2.0)*2.5E-1)-(nx*nx*nx*nx*nx)*pow(cos(shoulder),4.0)*3.244638711923428E+64+(ox*ox*ox*ox*ox)*pow(cos(shoulder),4.0)*4.633823435251701E+64+(nx*nx)*(ox*ox*ox)*pow(cos(shoulder),4.0)*9.267646870503402E+64-(nx*nx*nx)*(ox*ox)*pow(cos(shoulder),4.0)*6.489277423846856E+64-nx*(ox*ox*ox*ox)*pow(cos(shoulder),4.0)*3.244638711923428E+64+(nx*nx*nx*nx)*ox*pow(cos(shoulder),4.0)*4.633823435251701E+64-nx*(nz*nz*nz*nz)*pow(cos(base),4.0)*pow(sin(shoulder),4.0)*3.244638711923428E+64-nx*(oz*oz*oz*oz)*pow(cos(base),4.0)*pow(sin(shoulder),4.0)*3.244638711923428E+64+(nz*nz*nz*nz)*ox*pow(cos(base),4.0)*pow(sin(shoulder),4.0)*4.633823435251701E+64+ox*(oz*oz*oz*oz)*pow(cos(base),4.0)*pow(sin(shoulder),4.0)*4.633823435251701E+64-(nx*nx)*(nz*nz*nz)*pow(cos(base),3.0)*cos(shoulder)*pow(sin(shoulder),3.0)*1.297855484769371E+65+(ox*ox)*(oz*oz*oz)*pow(cos(base),3.0)*cos(shoulder)*pow(sin(shoulder),3.0)*1.85352937410068E+65-(nx*nx*nx*nx)*nz*cos(base)*pow(cos(shoulder),3.0)*sin(shoulder)*1.297855484769371E+65+(ox*ox*ox*ox)*oz*cos(base)*pow(cos(shoulder),3.0)*sin(shoulder)*1.85352937410068E+65-(nx*nx*nx)*(nz*nz)*pow(cos(base),2.0)*pow(cos(shoulder),2.0)*pow(sin(shoulder),2.0)*1.946783227154057E+65-(nx*nx*nx)*(oz*oz)*pow(cos(base),2.0)*pow(cos(shoulder),2.0)*pow(sin(shoulder),2.0)*6.489277423846856E+64+(nz*nz)*(ox*ox*ox)*pow(cos(base),2.0)*pow(cos(shoulder),2.0)*pow(sin(shoulder),2.0)*9.267646870503402E+64+(ox*ox*ox)*(oz*oz)*pow(cos(base),2.0)*pow(cos(shoulder),2.0)*pow(sin(shoulder),2.0)*2.780294061151021E+65-nx*(nz*nz)*(oz*oz)*pow(cos(base),4.0)*pow(sin(shoulder),4.0)*6.489277423846856E+64+(nz*nz)*ox*(oz*oz)*pow(cos(base),4.0)*pow(sin(shoulder),4.0)*9.267646870503402E+64+nx*nz*(ox*ox*ox)*cos(base)*pow(cos(shoulder),3.0)*sin(shoulder)*1.85352937410068E+65+(nx*nx*nx)*nz*ox*cos(base)*pow(cos(shoulder),3.0)*sin(shoulder)*1.85352937410068E+65-nx*(ox*ox*ox)*oz*cos(base)*pow(cos(shoulder),3.0)*sin(shoulder)*1.297855484769371E+65-(nx*nx*nx)*ox*oz*cos(base)*pow(cos(shoulder),3.0)*sin(shoulder)*1.297855484769371E+65-nx*(nz*nz)*(ox*ox)*pow(cos(base),2.0)*pow(cos(shoulder),2.0)*pow(sin(shoulder),2.0)*6.489277423846856E+64+(nx*nx)*(nz*nz)*ox*pow(cos(base),2.0)*pow(cos(shoulder),2.0)*pow(sin(shoulder),2.0)*2.780294061151021E+65-nx*(ox*ox)*(oz*oz)*pow(cos(base),2.0)*pow(cos(shoulder),2.0)*pow(sin(shoulder),2.0)*1.946783227154057E+65+(nx*nx)*ox*(oz*oz)*pow(cos(base),2.0)*pow(cos(shoulder),2.0)*pow(sin(shoulder),2.0)*9.267646870503402E+64-(nx*nx)*nz*(ox*ox)*cos(base)*pow(cos(shoulder),3.0)*sin(shoulder)*1.297855484769371E+65+nx*(nz*nz*nz)*ox*pow(cos(base),3.0)*cos(shoulder)*pow(sin(shoulder),3.0)*1.85352937410068E+65+(nx*nx)*(ox*ox)*oz*cos(base)*pow(cos(shoulder),3.0)*sin(shoulder)*1.85352937410068E+65-nx*ox*(oz*oz*oz)*pow(cos(base),3.0)*cos(shoulder)*pow(sin(shoulder),3.0)*1.297855484769371E+65-(nx*nx)*nz*(oz*oz)*pow(cos(base),3.0)*cos(shoulder)*pow(sin(shoulder),3.0)*1.297855484769371E+65+(nz*nz)*(ox*ox)*oz*pow(cos(base),3.0)*cos(shoulder)*pow(sin(shoulder),3.0)*1.85352937410068E+65+nx*nz*(ox*ox)*oz*pow(cos(base),2.0)*pow(cos(shoulder),2.0)*pow(sin(shoulder),2.0)*3.707058748201361E+65-(nx*nx)*nz*ox*oz*pow(cos(base),2.0)*pow(cos(shoulder),2.0)*pow(sin(shoulder),2.0)*2.595710969538743E+65+nx*nz*ox*(oz*oz)*pow(cos(base),3.0)*cos(shoulder)*pow(sin(shoulder),3.0)*1.85352937410068E+65-nx*(nz*nz)*ox*oz*pow(cos(base),3.0)*cos(shoulder)*pow(sin(shoulder),3.0)*1.297855484769371E+65)*1.25E-65)/((nz*nz*nz*nz)*pow(cos(base),5.0)*pow(sin(shoulder),5.0)*1.414213562373095+(oz*oz*oz*oz)*pow(cos(base),5.0)*pow(sin(shoulder),5.0)*1.414213562373095+(nx*nx*nx*nx)*cos(base)*pow(cos(shoulder),4.0)*sin(shoulder)*1.414213562373095+(ox*ox*ox*ox)*cos(base)*pow(cos(shoulder),4.0)*sin(shoulder)*1.414213562373095+(nz*nz)*(oz*oz)*pow(cos(base),5.0)*pow(sin(shoulder),5.0)*2.82842712474619+(nx*nx*nx)*nz*pow(cos(base),2.0)*pow(cos(shoulder),3.0)*pow(sin(shoulder),2.0)*5.65685424949238+(ox*ox*ox)*oz*pow(cos(base),2.0)*pow(cos(shoulder),3.0)*pow(sin(shoulder),2.0)*5.65685424949238+(nx*nx)*(nz*nz)*pow(cos(base),3.0)*pow(cos(shoulder),2.0)*pow(sin(shoulder),3.0)*8.48528137423857+(nx*nx)*(oz*oz)*pow(cos(base),3.0)*pow(cos(shoulder),2.0)*pow(sin(shoulder),3.0)*2.82842712474619+(nz*nz)*(ox*ox)*pow(cos(base),3.0)*pow(cos(shoulder),2.0)*pow(sin(shoulder),3.0)*2.82842712474619+(ox*ox)*(oz*oz)*pow(cos(base),3.0)*pow(cos(shoulder),2.0)*pow(sin(shoulder),3.0)*8.48528137423857+nx*(nz*nz*nz)*pow(cos(base),4.0)*cos(shoulder)*pow(sin(shoulder),4.0)*5.65685424949238+(nx*nx)*(ox*ox)*cos(base)*pow(cos(shoulder),4.0)*sin(shoulder)*2.82842712474619+ox*(oz*oz*oz)*pow(cos(base),4.0)*cos(shoulder)*pow(sin(shoulder),4.0)*5.65685424949238+nx*nz*(oz*oz)*pow(cos(base),4.0)*cos(shoulder)*pow(sin(shoulder),4.0)*5.65685424949238+(nz*nz)*ox*oz*pow(cos(base),4.0)*cos(shoulder)*pow(sin(shoulder),4.0)*5.65685424949238+nx*nz*(ox*ox)*pow(cos(base),2.0)*pow(cos(shoulder),3.0)*pow(sin(shoulder),2.0)*5.65685424949238+(nx*nx)*ox*oz*pow(cos(base),2.0)*pow(cos(shoulder),3.0)*pow(sin(shoulder),2.0)*5.65685424949238+nx*nz*ox*oz*pow(cos(base),3.0)*pow(cos(shoulder),2.0)*pow(sin(shoulder),3.0)*1.131370849898476E+1);
  double elbow = atan2(elbowSin, elbowCos);

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
