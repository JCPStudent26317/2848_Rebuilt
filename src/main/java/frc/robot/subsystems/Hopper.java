package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import static frc.robot.Constants.HopperConstants.*;

public class Hopper extends SubsystemBase {
    private final TalonFX beltMotor = new TalonFX(kBeltMotorID);
    private final TalonFXConfiguration beltMotorConfig = new TalonFXConfiguration();
    private final DutyCycleOut beltOut = new DutyCycleOut(0.0);
    
    private final TalonFX rollerMotor = new TalonFX(kRollerMotorID);
    private final TalonFXConfiguration rollerMotorConfig = new TalonFXConfiguration();
    private final DutyCycleOut rollerOut = new DutyCycleOut(0.0);

    public Hopper() {
        // Apply things to the configurations here

        beltMotor.getConfigurator().apply(beltMotorConfig);
        rollerMotor.getConfigurator().apply(rollerMotorConfig);
    }

    public Command holdState() {
        // setControl needs to run periodically at all times or the motor will disable I think
        return Commands.run(() -> {
            beltMotor.setControl(beltOut);
            rollerMotor.setControl(rollerOut);
        });
    }    

    public void setMotorsOutput(double beltOutput, double rollerOutput) {
        beltOut.Output = beltOutput;
        rollerOut.Output = rollerOutput;
    }

    public Command forward() {
        return Commands.runOnce(() -> setMotorsOutput(kBeltMotorSpeed, kRollerMotorSpeed));
    }

    public Command backward() {
        return Commands.runOnce(() -> setMotorsOutput(kBeltMotorSpeed * -1, kRollerMotorSpeed * -1));
    }

    public Command stop() {
        return Commands.runOnce(() -> setMotorsOutput(0.0, 0.0));
    }

}
