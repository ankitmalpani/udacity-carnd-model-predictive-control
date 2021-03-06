#ifndef MPC_UTILS_H
#define MPC_UTILS_H

#include <cmath>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include <cppad/cppad.hpp>

using namespace Eigen;
using namespace std;

struct Waypoint {
    double x;
    double y;
};

struct Waypoints {
    vector<double> x;
    vector<double> y;
};

typedef Waypoints AbsoluteWaypoints;
typedef Waypoints RelativeWaypoints;

double deg2rad(double x);

double rad2deg(double x);

const double deg2rad25 = deg2rad(25);

double polyeval(VectorXd coeffs, double x);

CppAD::AD<double> polyeval(VectorXd coeffs, CppAD::AD<double> x);

VectorXd derivative(VectorXd coeffs);

VectorXd vecXd(vector<double> vec);

VectorXd polyfit(VectorXd xvals, VectorXd yvals, int order);

RelativeWaypoints transformToRelative(AbsoluteWaypoints absolute, Waypoint origin, double psi);

double to_meters_per_second(double miles_per_hour);

double to_miles_per_hour(double meters_per_second);

#endif //MPC_UTILS_H
