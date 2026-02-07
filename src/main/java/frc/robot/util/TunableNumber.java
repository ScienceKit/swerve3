package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TunableNumber {
  private final String key;
  private final double defaultValue;
  private double value;

  public TunableNumber(String key) {
    this(key, 0, false);
  }

  public TunableNumber(String key, double defaultValue) {
    this(key, defaultValue, true);
  }

  // change the reset logic, use initdouble or smth
  public TunableNumber(String key, double defaultValue, boolean reset) {
    this.key = key;
    this.defaultValue = defaultValue;

    if (reset || !Dashboard.containsKey(key)) Dashboard.setNumber(key, defaultValue);
    this.value = Dashboard.getNumber(key, defaultValue);
  }

  public boolean changed() {
    return Dashboard.getNumber(key, defaultValue) != value;
  }

  public double get() {
    update();
    return value;
  }

  public void set(double val) {
    Dashboard.setNumber(key, val);
  }

  private void update() {
    value = Dashboard.getNumber(key, defaultValue);
  }

  private static class Dashboard {
    // public static double getNumber(String key, double def) { return Preferences.getDouble(key,
    // def); }
    // public static void setNumber(String key, double number) {  Preferences.setDouble(key,
    // number); }
    // public static boolean containsKey(String key) { return Preferences.containsKey(key); }

    public static double getNumber(String key, double def) {
      return SmartDashboard.getNumber(key, def);
    }

    public static void setNumber(String key, double number) {
      SmartDashboard.putNumber(key, number);
    }

    public static boolean containsKey(String key) {
      return SmartDashboard.containsKey(key);
    }
  }
}
