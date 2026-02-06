package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ShooterConstants.*;
import static frc.robot.RangerHelpers.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MotorOutputConfigs;

import lombok.Getter;

/** Shooter Subsystem. */
public class Shooter extends SubsystemBase {
  private final TalonFX m_FlywheelLeftLeader;
  private final TalonFX m_FlywheelRightFollower;

  private final TalonFX m_Turret;
  private final CANcoder m_TurretCANcoder;

  //private final TalonFX m_Hood;

  private final VelocityVoltage m_FlywheelVV = new VelocityVoltage(0).withSlot(0);
  private final DutyCycleOut m_FlywheelOut = new DutyCycleOut(0.0);

  private final PositionVoltage m_TurretPV = new PositionVoltage(0).withSlot(0);

  //private final PositionVoltage m_HoodVoltage = new PositionVoltage(0).withSlot(0);

  private @Getter double m_FlywheelOutputDutyCycle = 0;
  private @Getter long m_TurretAngle = 0; // Use Radians, 0 is from the front of the robot


  /** Shooter Subsystem. */
  public Shooter() {
    m_FlywheelLeftLeader = new TalonFX(kFlywheelLeftMotorID);
    m_FlywheelRightFollower = new TalonFX(kFlywheelRightMotorID);
    m_Turret = new TalonFX(kTurretMotorID);
    m_TurretCANcoder = new CANcoder(kTurretCANcoderID);
    //m_Hood = new TalonFX(kHoodMotorID);

     

    // All the FF and PID constants should be moved to constants once they are determined
    TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
    flywheelConfig.Slot0.kS = 0;
    flywheelConfig.Slot0.kV = 0;
    flywheelConfig.Slot0.kP = .000175;  
    flywheelConfig.Slot0.kI = 0.0;
    flywheelConfig.Slot0.kD = 0.0;
    flywheelConfig.Voltage.withPeakForwardVoltage(Volts.of(8)).withPeakReverseVoltage(Volts.of(-8));
    flywheelConfig.MotorOutput.Inverted = new MotorOutputConfigs().Inverted.Clockwise_Positive;
    setupTalonFx(m_FlywheelLeftLeader, flywheelConfig);
    m_FlywheelRightFollower.setControl(new Follower(m_FlywheelLeftLeader.getDeviceID(), MotorAlignmentValue.Opposed));

    TalonFXConfiguration turretConfig = new TalonFXConfiguration();
    turretConfig.Slot0.kS = 0;
    turretConfig.Slot0.kV = 0;
    turretConfig.Slot0.kP = 0;
    turretConfig.Slot0.kI = 0;
    turretConfig.Slot0.kD = 0;
    turretConfig.Voltage
        .withPeakForwardVoltage(Volts.of(6))
        .withPeakReverseVoltage(Volts.of(-6));
    turretConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    turretConfig.Feedback.FeedbackRemoteSensorID = kTurretCANcoderID;
    setupTalonFx(m_Turret, turretConfig);
    CANcoderConfigurator turretCANcoderConfigurator = m_TurretCANcoder.getConfigurator();
    retryConfigApply(() -> turretCANcoderConfigurator.apply(kTurretCANcoderMagnetSensorConfigs));

    TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
    hoodConfig.Slot0.kS = 0;
    hoodConfig.Slot0.kV = 0;
    hoodConfig.Slot0.kP = 0;
    hoodConfig.Slot0.kI = 0;
    hoodConfig.Slot0.kD = 0;
    hoodConfig.Voltage
        .withPeakForwardVoltage(Volts.of(6))
        .withPeakReverseVoltage(Volts.of(-6));
    //setupTalonFx(m_Hood, hoodConfig);
    //m_FlywheelLeftLeader.setControl(flywheelOut);
    init();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Flywheel RPM",getFlywheelRPM());
    SmartDashboard.putNumber("Flywheel vel error",getFlywheelRPM()-getM_FlywheelOutputDutyCycle());
    SmartDashboard.putData(this);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addDoubleProperty("Flywheel Output Duty Cycle",
        this::getM_FlywheelOutputDutyCycle,
        this::setM_FlywheelOutputDutyCycle);
    builder.addIntegerProperty("m_TurretAngle", 
        this::getM_TurretAngle, 
        this::setM_TurretAngle);

    
  } 

  /** Sets the turret angle to aim the shooter at the target.*/
  private Command setTurret() {
    return Commands.runOnce(
      () -> m_Turret.setControl(m_TurretPV.withPosition(m_TurretAngle / 180)), 
      this);
  }

  /** Sets the hood angle.*/
  private Command setHood() {
    return null;
  }

public void init(){
  CommandScheduler.getInstance().schedule(setFlywheel());
}


  public double getFlywheelRPM(){
    return m_FlywheelLeftLeader.getVelocity().getValueAsDouble()*60;
  }
  /**
   * Sets the target velocity for the flywheel.
   *
   * @param vel desired flywheel velocity (units defined by motor configuration)
   */
  private Command setFlywheel() {
    return Commands.runOnce(
      () -> m_FlywheelLeftLeader.setControl(new VelocityDutyCycle(m_FlywheelOutputDutyCycle)),this);
  }

  public void flyWheelOn(){
    m_FlywheelOutputDutyCycle = 4000;
    CommandScheduler.getInstance().schedule(setFlywheel());
  }
  public void flyWheelOff(){
    m_FlywheelOutputDutyCycle = 0;
    CommandScheduler.getInstance().schedule(setFlywheel());
  }
  /** Set the target for the shooter. */
  public void setTarget() {

  }

  /** Sets the flywheel and hood angle to their shot velocity and shot position. */
  public Command shoot() {
    return holdState().alongWith(Commands.idle(this).onlyIf(() -> readyToShoot()).repeatedly());
  }

  /** Hold the current shooting state. */
  public Command holdState() {
    return setFlywheel().andThen(setTurret());
  }

  /**
   * Checks whether the shooter has reached its target aiming state.
   *
   * @return true when the shooter is aligned and ready to fire.
   */
  public boolean readyToShoot() {
    return false;
  }

  /** Sets the turret angle in degrees, clamped to [-135, 135]. */
  public void setM_TurretAngle(long angle) {
    m_TurretAngle = (long) MathUtil.clamp(angle, -135, 135);
  }

  /** Sets the flywheel output duty cycle, clamped to [-1, 1]. */
  public void setM_FlywheelOutputDutyCycle(double angle) {
    m_FlywheelOutputDutyCycle = MathUtil.clamp(angle, -1, 1);
  }

}