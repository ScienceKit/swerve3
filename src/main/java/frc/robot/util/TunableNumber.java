package frc.robot.util;

import edu.wpi.first.wpilibj.Preferences;

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

    if (reset || !Preferences.containsKey(key)) Preferences.setDouble(key, defaultValue);
    this.value = Preferences.getDouble(key, defaultValue);
  }

  public boolean changed() {
    return Preferences.getDouble(key, defaultValue) != value;
  }

  public double get() {
    update();
    return value;
  }

  public void set(double val) {
    Preferences.setDouble(key, val);
  }

  private void update() {
    value = Preferences.getDouble(key, defaultValue);
  }

}
