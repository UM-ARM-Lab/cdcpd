#include <cdcpd/optimizer.h>

int main()
{
  Matrix3Xf init_temp(3,5);
  init_temp(0, 0) = 1.0; init_temp(0, 1) = 0.0; init_temp(0, 2) = -1.0; // near: (sqrt(2)/2, 0, 0); normal: (1/sqrt(3), 0, sqrt(2/3))
  init_temp(0, 0) = 1.0; init_temp(0, 1) = 0.0; init_temp(0, 2) = -1.0;
  init_temp(0, 0) = 1.0; init_temp(0, 1) = 0.0; init_temp(0, 2) = -1.0;
  init_temp(0, 0) = 1.0; init_temp(0, 1) = 0.0; init_temp(0, 2) = -1.0;
  init_temp(0, 0) = 1.0; init_temp(0, 1) = 0.0; init_temp(0, 2) = -1.0;
}
