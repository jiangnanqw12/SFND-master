#include <iostream>
int main() {
  float gauss_data[25] = {1,  4, 7, 4,  1,  4,  16, 26, 16, 4, 7, 26, 41,
                          26, 7, 4, 16, 26, 16, 4,  1,  4,  7, 4, 1};
  float x = 0;
  for (float i : gauss_data) {
    x += i;

    // std::cout << i << std::endl;
  }
  std::cout << x << std::endl;
}