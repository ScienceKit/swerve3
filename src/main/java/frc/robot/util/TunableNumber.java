package frc.robot.util;

import edu.wpi.first.wpilibj.Preferences;

public class TunableNumber {
  private final String key;
  private final double defaultValue;
  private double value;

  public TunableNumber(String key) {
    this(key, 0);
  }

  public TunableNumber(String key, double defaultValue) {
    this(key, defaultValue, true);
  }

  public TunableNumber(String key, double defaultValue, boolean reset) {
    this.key = key;
    this.defaultValue = defaultValue;
    if (reset) Preferences.setDouble(key, defaultValue);

    // Dont set value directly because we want it to reflect what is in networktables.
    this.value = Preferences.getDouble(key, defaultValue);
  }

  public boolean changed() {
    return Preferences.getDouble(key, defaultValue) != value;
  }

  public void update() {
    value = Preferences.getDouble(key, defaultValue);
  }

  public double get() {
    return value;
  }

  public void set(double val) {
    Preferences.setDouble(key, val);
    get(); // Nasko I put this here to update value. So we can see if NT fucks up.
  }
}
