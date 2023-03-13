#include <Eigen/Dense>
#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>

#include "dataStructures.h"
#include "structIO.hpp"

using namespace std;

// Helper function to fit a second-order polynomial to the data and return the
// quadratic coefficient
double polyfit(std::vector<double> x, double dt) {
  int n = x.size();
  Eigen::MatrixXd X(n, 3);
  Eigen::VectorXd y(n);

  for (int i = 0; i < n; i++) {
    X(i, 0) = 1.0;
    X(i, 1) = x[i];
    X(i, 2) = x[i] * x[i];
    y(i) = i * dt;
  }

  Eigen::VectorXd coeffs = X.colPivHouseholderQr().solve(y);

  return coeffs(2);
}

void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double &TTC) {
  // auxiliary variables
  double dT = 0.1;        // time between two measurements in seconds
  double laneWidth = 4.0; // assumed width of the ego lane
  double minSpeed = 0.1;  // minimum speed to avoid division by zero

  // find closest distance to Lidar points within ego lane
  double minXPrev = 1e9, minXCurr = 1e9;
  for (auto it = lidarPointsPrev.begin(); it != lidarPointsPrev.end(); ++it) {
    if (it->y >= -1 * laneWidth / 2 && it->y <= laneWidth / 2) {
      minXPrev = minXPrev > it->x ? it->x : minXPrev;
    }
  }

  for (auto it = lidarPointsCurr.begin(); it != lidarPointsCurr.end(); ++it) {
    if (it->y >= -1 * laneWidth / 2 && it->y <= laneWidth / 2) {
      minXCurr = minXCurr > it->x ? it->x : minXCurr;
    }
  }

  // compute TTC using constant acceleration model
  double acc = 0.0;
  double vel = (minXPrev - minXCurr) / dT;
  if (vel > minSpeed) {
    // estimate acceleration using quadratic fit
    std::vector<double> xPrev, xCurr;
    for (auto it = lidarPointsPrev.begin(); it != lidarPointsPrev.end(); ++it) {
      if (it->y >= -1 * laneWidth / 2 && it->y <= laneWidth / 2) {
        xPrev.push_back(it->x);
      }
    }

    for (auto it = lidarPointsCurr.begin(); it != lidarPointsCurr.end(); ++it) {
      if (it->y >= -1 * laneWidth / 2 && it->y <= laneWidth / 2) {
        xCurr.push_back(it->x);
      }
    }

    if (xPrev.size() > 2 && xCurr.size() > 2) {
      double prevA = polyfit(xPrev, dT);
      double currA = polyfit(xCurr, dT);
      acc = (currA - prevA) / dT;
    }
  }

  // compute TTC using constant acceleration
  TTC = minXCurr / (vel / acc);
}

int main() {

  std::vector<LidarPoint> currLidarPts, prevLidarPts;
  readLidarPts("../dat/C22A5_currLidarPts.dat", currLidarPts);
  readLidarPts("../dat/C22A5_prevLidarPts.dat", prevLidarPts);

  double ttc;
  computeTTCLidar(prevLidarPts, currLidarPts, ttc);
  cout << "TTC = " << ttc << " s" << endl;

  return 0;
}