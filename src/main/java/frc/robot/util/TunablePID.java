package frc.robot.util;

public class TunablePID {
    private TunableNumber tP;
    private TunableNumber tI;
    private TunableNumber tD;

    public TunablePID(String key, double p, double i, double d) {
        tP = new TunableNumber(key + "/p", p);
        tI = new TunableNumber(key + "/i", i);
        tD = new TunableNumber(key + "/d", d);
    }

    public TunablePID(String key) {
        this(key, 0, 0, 0);
    }

    public double p() { return tP.get(); }
    public double i() { return tI.get(); }
    public double d() { return tD.get(); }

    public boolean changed() {
        return tP.changed() || tI.changed() || tD.changed();
    }
}
