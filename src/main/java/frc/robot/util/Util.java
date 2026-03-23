package frc.robot.util;

import java.util.UUID;

public class Util {
  public static String randomString() {
    return "_temp/" + UUID.randomUUID().toString();
  }

  public static double remap(double v, double imin, double imax, double omin, double omax) {
    return omin + (v - imin) * (omax - omin) / (imax - imin);
  }
}
