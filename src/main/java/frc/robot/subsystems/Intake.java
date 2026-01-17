package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import static frc.robot.Constants.IntakeConstants.*;

public class Intake extends SubsystemBase {
    private final TalonFX intakeMotor = new TalonFX(kIntakeMotorID);
    private final TalonFXConfiguration intakeMotorConfig = new TalonFXConfiguration();
    private final DutyCycleOut intakeOut = new DutyCycleOut(0.0);

    public Intake() {
        // Apply things to the configuration here

        intakeMotor.getConfigurator().apply(intakeMotorConfig);
    }

    public Command holdState() {
        // setControl needs to run periodically at all times or the motor will disable I think
        return Commands.run(() -> intakeMotor.setControl(intakeOut));
    }

    public void setMotorOutput(double output) {
        intakeOut.Output = output;
    }

    public Command intake() {
        return Commands.runOnce(() -> setMotorOutput(1.0));
    }

    public Command outtake() {
        return Commands.runOnce(() -> setMotorOutput(-1.0));
    }

    public Command stop() {
        return Commands.runOnce(() -> setMotorOutput(0.0));
    }

    // Put these on a whiletrue connected to a controller button
    public Command manualIntake() {
        return Commands.startRun(() -> setMotorOutput(1.0), () -> intakeMotor.setControl(intakeOut))
        .finallyDo(() -> setMotorOutput(0.0));
    }

    public Command manualOuttake() {
        return Commands.startRun(() -> setMotorOutput(-1.0), () -> intakeMotor.setControl(intakeOut))
        .finallyDo(() -> setMotorOutput(0.0));
    }

}
