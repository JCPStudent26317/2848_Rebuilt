package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.Enable5VRailValue;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.LightsConstants.*;

public class Lights extends SubsystemBase {
    
    private final CANdle candle = new CANdle(kCANdleID);
    private final CANdleConfiguration candleconfig = new CANdleConfiguration();

    private ControlRequest candleRequest = kRainbow; 

    public Lights() {
        candleconfig.LED.StripType = StripTypeValue.RGB;
        candleconfig.LED.BrightnessScalar = 1;
        candleconfig.CANdleFeatures.Enable5VRail = Enable5VRailValue.Enabled;
        
        candle.getConfigurator().apply(candleconfig);
    }

    @Override
    public void periodic() {
        candle.setControl(candleRequest);

        
    }

    public Command runPattern(ControlRequest request) {
        return Commands.runOnce(() -> candleRequest = request, this);
    }

}
