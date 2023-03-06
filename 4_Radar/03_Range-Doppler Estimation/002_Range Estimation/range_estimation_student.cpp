#include <cmath>
#include <iostream>
#include <vector>

using namespace std;

int main() {
  // Define constants
  double c = 3e8;
  double resolution = 1;
  double range_max = 300;

  // Calculate the bandwidth of the chirp signal
  double Bsweep = c / (2 * resolution);

  // Calculate the chirp time
  double Ts = 5.5 * (2 * range_max / c);

  // Define the frequency shifts
  vector<double> fb{0, 1.1e6, 13e6, 24e6};

  // Calculate the range
  vector<double> calculated_range(fb.size());
  for (int i = 0; i < fb.size(); i++) {
    calculated_range[i] = c * Ts * fb[i] / (2 * Bsweep);
  }

  // Display the calculated range
  for (int i = 0; i < fb.size(); i++) {
    cout << calculated_range[i] << " ";
  }
  cout << endl;

  return 0;
}
