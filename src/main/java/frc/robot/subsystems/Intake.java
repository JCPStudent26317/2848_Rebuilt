package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.RangerHelpers.*;

public class Intake extends SubsystemBase {
    private final TalonFX lRollersMotor = new TalonFX(kLRollersMotorID);
    private final TalonFXConfiguration lRollersMotorConfig = new TalonFXConfiguration();
    private final DutyCycleOut lRollersOut = new DutyCycleOut(0.0);

    private final TalonFX rRollersMotor = new TalonFX(kRRollersMotorID);
    private final TalonFXConfiguration rRollersMotorConfig = new TalonFXConfiguration();

    private final TalonFX pivotMotor = null;
    private final TalonFXConfiguration pivotMotorConfig = new TalonFXConfiguration();
    // Pivot motor would use PID, maybe feedforward with arm angle

    public Intake() {
        // Apply things to the configuration here
        lRollersMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        setupTalonFx(lRollersMotor, lRollersMotorConfig);
        setupTalonFx(rRollersMotor, rRollersMotorConfig);
        setupTalonFx(pivotMotor, pivotMotorConfig);
    
        rRollersMotor.setControl(new Follower(lRollersMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    } 

    public Command holdState() {
        // setControl needs to run periodically at all times or the motor will disable I think
        return Commands.run(() -> lRollersMotor.setControl(lRollersOut),this);
    }

    public void setMotorOutput(double output) {
        lRollersOut.Output = output;
    }

    public Command intake() {
        return Commands.runOnce(() -> setMotorOutput(kRollersMotorSpeed));
    }

    public Command outtake() {
        return Commands.runOnce(() -> setMotorOutput(kRollersMotorSpeed * -1));
    }

    public Command stop() {
        return Commands.runOnce(() -> setMotorOutput(0.0));
    }

}
