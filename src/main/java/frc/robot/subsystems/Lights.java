package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase {
    
    private final AddressableLED led = new AddressableLED(0);    
    private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(20);

    public Lights() {
        led.setLength(ledBuffer.getLength());
        led.start();

        setDefaultCommand(runPattern(LEDPattern.solid(Color.kBlack)));
    }

    @Override
    public void periodic() {
        led.setData(ledBuffer);
    }

    public Command runPattern(LEDPattern pattern) {
        return run(() -> pattern.applyTo(ledBuffer));
    }

}
