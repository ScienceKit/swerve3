package frc.robot.subsystems.LED_bangarana;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LEDController implements AutoCloseable{

    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_ledBuffer;
    private LEDPattern currentPattern = LEDConstants.pLEDsDisabled;

    public LEDController()
    {
        m_led = new AddressableLED(LEDConstants.kLEDPWMPort);
        m_ledBuffer = new AddressableLEDBuffer(LEDConstants.kLEDCount);
        m_led.setLength(LEDConstants.kLEDCount);

        // Set the data
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    public void setPattern(LEDPattern newPattern)
    {
        this.currentPattern = newPattern;
    }

    public void disable()
    {
        setPattern(LEDConstants.pLEDsDisabled);
    }

    public void periodic()
    {
        currentPattern.applyTo(m_ledBuffer);
        m_led.setData(m_ledBuffer);
        SmartDashboard.putNumber("ColorTest", m_ledBuffer.getBlue(0));
    }

    @Override
    public void close() {
        setPattern(LEDConstants.pLEDsDisabled);
        m_led.stop();
        m_led.close();
    } 
}
