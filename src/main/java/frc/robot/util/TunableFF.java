package frc.robot.util;

public class TunableFF {
  public static record FFConstants(double kS, double kV, double kA, double kG, double kCos) {}

  private final TunableNumber tkS;
  private final TunableNumber tkV;
  private final TunableNumber tkA;
  private final TunableNumber tkG;
  private final TunableNumber tkCos;

  public TunableFF(String key) {
    this(key, 0, 0, 0, 0, 0);
  }

  public TunableFF(String key, double kS, double kV, double kA, double kG, double kCos) {
    this(key, kS, kV, kA, kG, kCos, true);
  }

  public TunableFF(
      String key, double kS, double kV, double kA, double kG, double kCos, boolean reset) {
    tkS = new TunableNumber(key + "/kS", kS, reset);
    tkV = new TunableNumber(key + "/kV", kV, reset);
    tkA = new TunableNumber(key + "/kA", kA, reset);
    tkG = new TunableNumber(key + "/kG", kG, reset);
    tkCos = new TunableNumber(key + "/kCos", kCos, reset);
  }

  public boolean changed() {
    return tkS.changed() || tkV.changed() || tkA.changed() || tkG.changed() || tkCos.changed();
  }

  public FFConstants ff() {
    return new FFConstants(tkS.get(), tkV.get(), tkA.get(), tkG.get(), tkCos.get());
  }
}
