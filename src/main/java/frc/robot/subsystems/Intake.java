package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private final TalonFX m_RollersL = new TalonFX(kLRollersMotorID);
    private final TalonFXConfiguration lRollersMotorConfig = new TalonFXConfiguration();
    private final DutyCycleOut lRollersOut = new DutyCycleOut(0.0);

    private final TalonFX m_RollersR = new TalonFX(kRRollersMotorID);
    private final TalonFXConfiguration rRollersMotorConfig = new TalonFXConfiguration();

    private final TalonFX m_Pivot = new TalonFX(kIntakePivotID);
    private final TalonFXConfiguration pivotMotorConfig = new TalonFXConfiguration();

    private final CANcoder m_IntakeCANcoder = new CANcoder(kIntakePivotCANcoderID);

    private MotionMagicVoltage pivotOut = new MotionMagicVoltage(kDeploySetpoint);

    private double pivotSetpoint = kDeploySetpoint;
    // Pivot motor would use PID, maybe feedforward with arm angle

    public Intake() {

        pivotMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        pivotMotorConfig.Slot0.kS = 0;
        pivotMotorConfig.Slot0.kV = 0;
        pivotMotorConfig.Slot0.kA = 0;
        pivotMotorConfig.Slot0.kP = 0;//1;
        pivotMotorConfig.Slot0.kI = 0;
        pivotMotorConfig.Slot0.kD = 0;
        pivotMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 7;
        pivotMotorConfig.MotionMagic.MotionMagicAcceleration = 14;
        pivotMotorConfig.MotionMagic.MotionMagicJerk = 140;

        pivotMotorConfig.Feedback.FeedbackRemoteSensorID = kIntakePivotCANcoderID;
        pivotMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

        SoftwareLimitSwitchConfigs pivotSoftwareConfigs = new SoftwareLimitSwitchConfigs();
        pivotSoftwareConfigs.ForwardSoftLimitThreshold =-0.5;
        pivotSoftwareConfigs.ReverseSoftLimitThreshold =0;
        pivotSoftwareConfigs.ForwardSoftLimitEnable = true;
        pivotSoftwareConfigs.ReverseSoftLimitEnable = true;

        CANcoderConfigurator intakeCANcoderConfigurator = m_IntakeCANcoder.getConfigurator();
        retryConfigApply(() -> intakeCANcoderConfigurator.apply(kIntakeCANcoderMagnetSensorConfigs));

        // Apply things to the configuration here
        lRollersMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        setupTalonFx(m_RollersL, lRollersMotorConfig);
        setupTalonFx(m_RollersR, rRollersMotorConfig);
        setupTalonFx(m_Pivot, pivotMotorConfig);

        m_Pivot.getConfigurator().apply(pivotSoftwareConfigs);
    
        m_RollersR.setControl(new Follower(m_RollersL.getDeviceID(), MotorAlignmentValue.Opposed));
    } 

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pivot Motor Voltage", m_Pivot.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Pivot RPM", m_Pivot.getVelocity().getValueAsDouble() * 60);
        SmartDashboard.putNumber("Pivot Motor Temperature", m_Pivot.getDeviceTemp().getValueAsDouble());
        SmartDashboard.putNumber("Pivot Supply Current", m_Pivot.getSupplyCurrent().getValueAsDouble());

        SmartDashboard.putNumber("CANcoder Absolute Position", m_IntakeCANcoder.getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("Setpoint", pivotSetpoint);
    }

    public Command holdState() {
        // setControl needs to run periodically at all times or the motor will disable I think
        return Commands.run(() -> {
            m_RollersL.setControl(lRollersOut);
            m_Pivot.setControl(pivotOut);
        } ,this);
    }

    public void setPivot(double pos){
        pivotSetpoint = pos;
        pivotOut.Position = pos;
    }

    public void setRollersOutput(double output) {
        lRollersOut.Output = output;
    }

    public Command intake() {
        return Commands.runOnce(() -> setRollersOutput(kRollersMotorSpeed));
    }

    public Command outtake() {
        return Commands.runOnce(() -> setRollersOutput(kRollersMotorSpeed * -1));
    }

    public Command stop() {
        return Commands.runOnce(() -> setRollersOutput(0.0));
    }

    public Command deploy() {
        return Commands.runOnce(() -> setPivot(kDeploySetpoint));
    }

    public Command lowStow() {
        return Commands.runOnce(() -> setPivot(kLowStowSetpoint));
    }

    public Command highStow() {
        return Commands.runOnce(() -> setPivot(kHighStowSetpoint));
    }

}
