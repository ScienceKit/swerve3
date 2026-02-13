package frc.robot.util;

public class TunableFF {
  public static record FFConstants(double kS, double kV, double kA, double kG) {}

  private final TunableNumber tkS;
  private final TunableNumber tkV;
  private final TunableNumber tkA;
  private final TunableNumber tkG;

  public TunableFF(String key, double kS, double kV, double kA, double kG, boolean reset) {
    tkS = new TunableNumber(key + "/kS", kS);
    tkV = new TunableNumber(key + "/kV", kV);
    tkA = new TunableNumber(key + "/kA", kA);
    tkG = new TunableNumber(key + "/kG", kG);
  }

  public boolean changed() {
    return tkS.changed() || tkV.changed() || tkA.changed() || tkG.changed();
  }

  public FFConstants ff() {
    return new FFConstants(tkS.get(), tkV.get(), tkA.get(), tkG.get());
  }
}
