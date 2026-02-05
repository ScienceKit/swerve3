package frc.robot.util;

public class TunablePID {
  private final TunableNumber tP;
  private final TunableNumber tI;
  private final TunableNumber tD;

  public TunablePID(String key) {
    this(key, 0, 0, 0);
  }

  public TunablePID(String key, double p, double i, double d) {
    this(key, p, i, d, false);
  }

  public TunablePID(String key, double p, double i, double d, boolean reset) {
    tP = new TunableNumber(key + "/p", p, reset);
    tI = new TunableNumber(key + "/i", i, reset);
    tD = new TunableNumber(key + "/d", d, reset);
  }

  public double p() {
    return tP.get();
  }

  public double i() {
    return tI.get();
  }

  public double d() {
    return tD.get();
  }

  public boolean changed() {
    return tP.changed() || tI.changed() || tD.changed();
  }
}
