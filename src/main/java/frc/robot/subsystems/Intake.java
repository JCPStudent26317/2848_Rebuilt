package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
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

    private final TalonFX pivotMotor = new TalonFX(kIntakePivotID);
    private final TalonFXConfiguration pivotMotorConfig = new TalonFXConfiguration();

    private final CANcoder m_IntakeCANcoder = new CANcoder(kIntakePivotCANcoderID);

    private MotionMagicVoltage pivotOut = new MotionMagicVoltage(0);

    private double pivotSetpoint = 0;
    // Pivot motor would use PID, maybe feedforward with arm angle

    public Intake() {

        pivotMotorConfig.Slot0.kS = 0;
        pivotMotorConfig.Slot0.kV = 0;
        pivotMotorConfig.Slot0.kA = 0;
        pivotMotorConfig.Slot0.kP = 0;
        pivotMotorConfig.Slot0.kI = 0;
        pivotMotorConfig.Slot0.kD = 0;
        pivotMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 7;
        pivotMotorConfig.MotionMagic.MotionMagicAcceleration = 14;
        pivotMotorConfig.MotionMagic.MotionMagicJerk = 140;

        pivotMotorConfig.Feedback.FeedbackRemoteSensorID= kIntakePivotCANcoderID;
        pivotMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

        SoftwareLimitSwitchConfigs pivotSoftwareConfigs = new SoftwareLimitSwitchConfigs();
        pivotSoftwareConfigs.ForwardSoftLimitThreshold =1;
        pivotSoftwareConfigs.ReverseSoftLimitThreshold =0;
        pivotSoftwareConfigs.ForwardSoftLimitEnable = true;
        pivotSoftwareConfigs.ReverseSoftLimitEnable = true;

        CANcoderConfigurator intakeCANcoderConfigurator = m_IntakeCANcoder.getConfigurator();
        retryConfigApply(() -> intakeCANcoderConfigurator.apply(kIntakeCANcoderMagnetSensorConfigs));

        // Apply things to the configuration here
        lRollersMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        setupTalonFx(lRollersMotor, lRollersMotorConfig);
        setupTalonFx(rRollersMotor, rRollersMotorConfig);
        setupTalonFx(pivotMotor, pivotMotorConfig);

        pivotMotor.getConfigurator().apply(pivotSoftwareConfigs);
    
        rRollersMotor.setControl(new Follower(lRollersMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    } 


    public Command setPivot(){
        return new InstantCommand(()->pivotMotor.setControl(pivotOut.withPosition(pivotSetpoint)));
    }

    public Command holdState() {
        // setControl needs to run periodically at all times or the motor will disable I think
        return Commands.run(() -> lRollersMotor.setControl(lRollersOut),this);
    }

    public void setPivotOutput(double pos){
        pivotOut.Position = pos;
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
