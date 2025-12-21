#include <stdio.h>

void forcesToPWM(const double forces[6], double u_mapped[8]) {
  // Precomputed pinv(B) (8x6)  [cols: Fx Fy Fz Mx My Mz]
  static const double pinvB[8][6] = {
      {0.3432, 0.3432, 0.1458, -0.0060, -0.0057, 0.0069},
      {0.3432, -0.3432, 0.1458, 0.0060, -0.0057, -0.0069},
      {-0.3432, 0.3432, 0.1458, -0.0060, 0.0057, -0.0069},
      {-0.3432, -0.3432, 0.1458, 0.0060, 0.0057, 0.0069},
      {0.3432, 0.3432, -0.1458, 0.0060, 0.0057, 0.0069},
      {0.3432, -0.3432, -0.1458, -0.0060, 0.0057, -0.0069},
      {-0.3432, 0.3432, -0.1458, 0.0060, -0.0057, -0.0069},
      {-0.3432, -0.3432, -0.1458, -0.0060, -0.0057, 0.0069}};

  // 1) Compute u = pinvB * forces  (8x6 * 6x1 -> 8x1)
  double u[8];
  for (int i = 0; i < 8; ++i) {
    double acc = 0.0;
    for (int j = 0; j < 6; ++j) {
      acc += pinvB[i][j] * forces[j];
    }
    u[i] = acc;
  }

  // 2) Map from [-1, +1] -> [44, 76] then scale by 100 => [4400, 7600]
  const double in_min = -1.0, in_max = 1.0;
  const double out_min = 44.0, out_max = 76.0;

  for (int i = 0; i < 8; ++i) {
    double val = u[i];

    // clamp u to [-1,1]
    if (val > in_max)
      val = in_max;
    if (val < in_min)
      val = in_min;

    // linear map then scale x100
    double pwm =
        ((val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min) *
        100.0;

    // clamp mapped pwm to [4400,7600]
    if (pwm > 7600.0)
      pwm = 7600.0;
    if (pwm < 4400.0)
      pwm = 4400.0;

    u_mapped[i] = pwm;
  }
}

static void printPWM(const char *name, const double forces[6]) {
  double pwm[8];
  forcesToPWM(forces, pwm);

  printf("\n=== Test: %s ===\n", name);
  printf("forces = [Fx Fy Fz Mx My Mz] = [%.2f %.2f %.2f %.2f %.2f %.2f]\n",
         forces[0], forces[1], forces[2], forces[3], forces[4], forces[5]);

  for (int i = 0; i < 8; ++i) {
    printf("pwm[%d] = %.1f\n", i, pwm[i]);
  }
}

int main(void) {
  // Pure surge
  const double f1[6] = {1, 0, 0, 0, 0, 0};
  // Pure sway
  const double f2[6] = {0, 1, 0, 0, 0, 0};
  // Pure heave
  const double f3[6] = {0, 0, 1, 0, 0, 0};
  // Pure yaw
  const double f4[6] = {0, 0, 0, 0, 0, 4};
  // Mixed: sway + yaw (your style)
  const double f5[6] = {0, 1, 0, 0, 0, 10};
  // Mixed: surge + yaw
  const double f6[6] = {10, 0, 0, 0, 0, 10};

  printPWM("Surge only", f1);
  printPWM("Sway only", f2);
  printPWM("Heave only", f3);
  printPWM("Yaw only", f4);
  printPWM("Sway + 10*Yaw", f5);
  printPWM("10*Surge + 10*Yaw", f6);

  return 0;
}
