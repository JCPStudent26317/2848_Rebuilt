package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import static frc.robot.Constants.IntakeConstants.*;

public class Intake extends SubsystemBase {
    private final TalonFX intakeMotor = new TalonFX(kIntakeMotorID);
    private final TalonFXConfiguration intakeMotorConfig = new TalonFXConfiguration();
    private final DutyCycleOut intakeOut = new DutyCycleOut(0.0);

    private final TalonFX pivotMotor = null;
    private final TalonFXConfiguration pivotMotorConfig = new TalonFXConfiguration();
    // Pivot motor would use PID, maybe feedforward with arm angle

    public Intake() {
        // Apply things to the configuration here
        intakeMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        intakeMotor.getConfigurator().apply(intakeMotorConfig);
        pivotMotor.getConfigurator().apply(pivotMotorConfig);
    } 

    public Command holdState() {
        // setControl needs to run periodically at all times or the motor will disable I think
        return Commands.run(() -> intakeMotor.setControl(intakeOut),this);
    }

    public void setMotorOutput(double output) {
        intakeOut.Output = output;
    }

    public Command intake() {
        return Commands.runOnce(() -> setMotorOutput(kIntakeMotorSpeed));
    }

    public Command outtake() {
        return Commands.runOnce(() -> setMotorOutput(kIntakeMotorSpeed * -1));
    }

    public Command stop() {
        return Commands.runOnce(() -> setMotorOutput(0.0));
    }

}
