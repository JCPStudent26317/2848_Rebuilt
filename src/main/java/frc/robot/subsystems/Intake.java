package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

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

    private double pivotSetpoint = kStowSetpoint;
    private MotionMagicVoltage pivotOut = new MotionMagicVoltage(pivotSetpoint);

    // Pivot motor would use PID, maybe feedforward with arm angle

    public Intake() {

        pivotMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        pivotMotorConfig.Slot0.kS = kPivotkS;
        pivotMotorConfig.Slot0.kV = kPivotkV;
        pivotMotorConfig.Slot0.kA = kPivotkA;
        pivotMotorConfig.Slot0.kP = kPivotkP;
        pivotMotorConfig.Slot0.kI = kPivotkI;
        pivotMotorConfig.Slot0.kD = kPivotkD;
        pivotMotorConfig.MotionMagic.MotionMagicCruiseVelocity = kPivotMMCruiseVelocity;
        pivotMotorConfig.MotionMagic.MotionMagicAcceleration = kPivotMMAcceleration;
        pivotMotorConfig.MotionMagic.MotionMagicJerk = kPivotMMJerk;

        pivotMotorConfig.Feedback.FeedbackRemoteSensorID = kIntakePivotCANcoderID;
        pivotMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        //pivotMotorConfig.Feedback.SensorToMechanismRatio =-1;
        
        pivotMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =-0.5;
        pivotMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =0;
        pivotMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        pivotMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

        CANcoderConfigurator intakeCANcoderConfigurator = m_IntakeCANcoder.getConfigurator();
        retryConfigApply(() -> intakeCANcoderConfigurator.apply(kIntakeCANcoderMagnetSensorConfigs));


        // Apply things to the configuration here
        lRollersMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        setupTalonFx(m_RollersL, lRollersMotorConfig);
        setupTalonFx(m_RollersR, rRollersMotorConfig);
        setupTalonFx(m_Pivot, pivotMotorConfig);
    
        m_RollersR.setControl(new Follower(m_RollersL.getDeviceID(), MotorAlignmentValue.Opposed));

        m_IntakeCANcoder.setPosition(m_IntakeCANcoder.getAbsolutePosition().getValue());
    } 

    @Override
    public void periodic() {
        SmartDashboard.putData(this);

        // setControl needs to run periodically at all times or the motor will disable I think        
        m_RollersL.setControl(lRollersOut);
        m_Pivot.setControl(pivotOut);
    }

    public void changeStartPoint(double val){
        setPivot(val);
    }
    
    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addDoubleProperty("Pivot Motor Voltage", m_Pivot.getMotorVoltage()::getValueAsDouble, null);
        builder.addDoubleProperty("Pivot RPM", () -> m_Pivot.getVelocity().getValueAsDouble() * 60, null);
        builder.addDoubleProperty("Pivot Motor Temperature", m_Pivot.getDeviceTemp()::getValueAsDouble, null);
        builder.addDoubleProperty("Pivot Supply Current", m_Pivot.getSupplyCurrent()::getValueAsDouble, null);
        builder.addDoubleProperty("Pivot get()", m_Pivot::get, null);
        builder.addDoubleProperty("Pivot Error", m_Pivot.getClosedLoopError()::getValueAsDouble, null);

        builder.addDoubleProperty("CANcoder Absolute Position",() ->  m_IntakeCANcoder.getAbsolutePosition().getValueAsDouble(), null);
        builder.addDoubleProperty("CANcoder Non-absolute Position", () -> m_IntakeCANcoder.getPosition().getValueAsDouble(), null);        
        builder.addDoubleProperty("Pivot Motor Encoder Position",m_Pivot.getPosition()::getValueAsDouble, null);
        builder.addDoubleProperty("Setpoint", () -> pivotSetpoint, null);
    }

    public Command holdState() {
        return Commands.idle(this);
    }

    public void setPivot(double pos){
        pivotSetpoint = pos;
        pivotOut.Position = pos;
    }

    public void setRollersOutput(double output) {
        lRollersOut.Output = output;
    }

    public Command intake() {
        return Commands.runOnce(() -> setRollersOutput(kRollersMotorSpeed), this);
    }

    public Command outtake() {
        return Commands.runOnce(() -> setRollersOutput(kRollersMotorSpeed * -1), this);
    }

    public Command stop() {
        return Commands.runOnce(() -> setRollersOutput(0.0), this);
    }

    public Command deploy() {
        return Commands.runOnce(() -> setPivot(kDeploySetpoint), this);
    }

    public Command lowRetract() {
        return Commands.runOnce(() -> setPivot(kLowRetractSetpoint), this);
    }

    public Command highRetract() {
        return Commands.runOnce(() -> setPivot(kHighRetractSetpoint), this);
    }

    public Command stow() {
        return Commands.runOnce(() -> setPivot(kStowSetpoint), this);
    }

    public Command jiggle() {
        return new SequentialCommandGroup(
            highRetract(),
            new WaitCommand(0.25),
            lowRetract(),
            new WaitCommand(0.25)
        ).repeatedly().withName("Jiggle");
    }

    /**
     * Wires to the intake roller motors can get eaten by the transition when the intake is stowed.
     * This function outputs true if it is safe to run the transition motors.
     */
    public boolean safeToRunTransition() {
        return m_IntakeCANcoder.getAbsolutePosition().getValueAsDouble() <= kHighRetractSetpoint;
    }

}
