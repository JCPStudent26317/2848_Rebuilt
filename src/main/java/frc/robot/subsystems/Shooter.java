package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ShooterConstants.*;
import static frc.robot.RangerHelpers.*;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer;
import lombok.Getter;

/** Shooter Subsystem. */
public class Shooter extends SubsystemBase {
  private final TalonFX m_FlywheelLeftLeader;
  private final TalonFX m_FlywheelRightFollower;
  
  private final TalonFX m_Turret;
  private final CANcoder m_TurretCANcoder;

  private final TalonFX m_Magazine;

  private double targetDist =0;

  //private final TalonFX m_Hood;

  // private final VelocityVoltage m_FlywheelVV = new VelocityVoltage(0).withSlot(0);
  // private final DutyCycleOut m_FlywheelOut = new DutyCycleOut(0.0);

  // private final PositionVoltage m_TurretPV = new PositionVoltage(0).withSlot(0);

  private final MotionMagicVoltage turretOut = new MotionMagicVoltage(0);

  private double turretSetpoint = 0;

  private double hubTheta = 0;

  // create a Motion Magic request, voltage output


  //private final PositionVoltage m_HoodVoltage = new PositionVoltage(0).withSlot(0);

  private @Getter double m_FlywheelOutputDutyCycle = 0;
  private @Getter long m_TurretAngle = 0; // Use Radians, 0 is from the front of the robot


  /** Shooter Subsystem. */
  public Shooter() {
    m_FlywheelLeftLeader = new TalonFX(kFlywheelLeftMotorID);
    m_FlywheelRightFollower = new TalonFX(kFlywheelRightMotorID);
    m_Turret = new TalonFX(kTurretMotorID);
    m_TurretCANcoder = new CANcoder(kTurretCANcoderID);
    m_Magazine = new TalonFX(kMagazineMotorID);

    // All the FF and PID constants should be moved to constants once they are determined
    TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
    flywheelConfig.Slot0.kS = kFlywheelkS;
    flywheelConfig.Slot0.kV = kFlywheelkV;
    flywheelConfig.Slot0.kP = kFlywheelkP;//.000175;  
    flywheelConfig.Slot0.kI = kFlywheelkI;
    flywheelConfig.Slot0.kD = kFlywheelkD;
    flywheelConfig.Slot0.kA = kFlywheelkA;
    flywheelConfig.Voltage
        .withPeakForwardVoltage(Volts.of(kFlywheelPeakVoltage))
        .withPeakReverseVoltage(Volts.of(-1 * kFlywheelPeakVoltage));
    //flywheelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    setupTalonFx(m_FlywheelLeftLeader, flywheelConfig);
    m_FlywheelRightFollower.setControl(new Follower(m_FlywheelLeftLeader.getDeviceID(), MotorAlignmentValue.Opposed));

    TalonFXConfiguration turretConfig = new TalonFXConfiguration();
    turretConfig.Slot0.kS = kTurretkS;
    turretConfig.Slot0.kV = kTurretkV;
    turretConfig.Slot0.kA = kTurretkA;
    turretConfig.Slot0.kP = kTurretkP;
    turretConfig.Slot0.kI = kTurretkI;
    turretConfig.Slot0.kD = kTurretkD;
    turretConfig.Voltage
        .withPeakForwardVoltage(Volts.of(kTurretPeakVoltage))
        .withPeakReverseVoltage(Volts.of(-1 * kTurretPeakVoltage));
    turretConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    turretConfig.Feedback.FeedbackRemoteSensorID = kTurretCANcoderID;
    var motionMagicConfigs = turretConfig.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = kTurretMMCruiseVelocity; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration = kTurretMMAcceleration; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = kTurretMMJerk; // Target jerk of 1600 rps/s/s (0.1 seconds)
    setupTalonFx(m_Turret, turretConfig);

    SoftwareLimitSwitchConfigs turretSwitchConfigs = new SoftwareLimitSwitchConfigs();
    turretSwitchConfigs.ForwardSoftLimitThreshold = kTurretSwitchForwardLimit;
    turretSwitchConfigs.ReverseSoftLimitThreshold = kTurretSwitchReverseLimit;
    turretSwitchConfigs.ForwardSoftLimitEnable = true;
    turretSwitchConfigs.ReverseSoftLimitEnable = true;
    m_Turret.getConfigurator().apply(turretSwitchConfigs);
    
    //m_Turret.getConfigurator().apply(motionMagicConfigs);
    CANcoderConfigurator turretCANcoderConfigurator = m_TurretCANcoder.getConfigurator();
    retryConfigApply(() -> turretCANcoderConfigurator.apply(kTurretCANcoderMagnetSensorConfigs));

    TalonFXConfiguration magazineConfig = new TalonFXConfiguration();
    magazineConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    retryConfigApply(() -> m_Magazine.getConfigurator().apply(magazineConfig));

    init();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Flywheel RPM",getFlywheelRPM());
    SmartDashboard.putNumber("Flywheel vel error",getVeloRPM(getExitVelo())-getFlywheelRPM());
    SmartDashboard.putNumber("Commanded flywheel rpm",getVeloRPM(getExitVelo()));
    SmartDashboard.putNumber("Motor Output", m_FlywheelLeftLeader.get());
    SmartDashboard.putNumber("Shooter1 RPM",m_FlywheelLeftLeader.getVelocity().getValueAsDouble()/60);
    SmartDashboard.putNumber("Shooter2 RPM",m_FlywheelRightFollower.getVelocity().getValueAsDouble()/60);
    SmartDashboard.putNumber("turret angle",m_Turret.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("turret output",m_Turret.get());
    SmartDashboard.putNumber("turret error",m_Turret.getPosition().getValueAsDouble()-turretSetpoint);
    //turretSetpoint = MathUtil.clamp(SmartDashboard.getNumber("turret setpoint",0),-.33,.09);
    SmartDashboard.putNumber("turret setpoint",turretSetpoint);
    SmartDashboard.putData(this);
    SmartDashboard.putNumber("Shooter hub theta",hubTheta);


    hubTheta=(RobotContainer.getDrivetrain().getTargetTheta());
    targetDist = RobotContainer.getDrivetrain().getTargetDist();
    setTurretAngle(hubTheta);
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
  public Command setTurret() {
    return Commands.runOnce(
      () -> {},//m_Turret.setControl(turretOut.withPosition(turretSetpoint)), 
      this);
  }


public void init(){
  //CommandScheduler.getInstance().schedule(setFlywheel());
  SmartDashboard.putNumber("turret setpoint",turretSetpoint);
  SmartDashboard.putNumber("Distance",targetDist);
}


private double getExitVelo(){
  return  0.6632 * targetDist + 5.7486;
}

private double getVeloRPM(double velo){
  return (velo*60)/(2*Math.PI*kFlywheelRadius) * kFlywheelRPMMult;
}

public double getTurretAngle(){
  return m_TurretCANcoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI;
}


public void setTurretAngle(double angle){
  turretSetpoint = angle / (2 * Math.PI);
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
      () -> m_FlywheelLeftLeader.setControl(new VelocityVoltage(m_FlywheelOutputDutyCycle)),this);
  }

  public void flyWheelOn(){
    //m_FlywheelOutputDutyCycle = getVeloRPM(getExitVelo());
    m_FlywheelOutputDutyCycle = 4000;
    CommandScheduler.getInstance().schedule(setFlywheel());
  }
  public void flyWheelOff(){
    m_FlywheelOutputDutyCycle = 0;
    CommandScheduler.getInstance().schedule(setFlywheel());
  }
  // /** Set the target for the shooter. */
  // public void setTarget() {

  // }

  /** Sets the flywheel and hood angle to their shot velocity and shot position. */
  public Command shoot() {
    return holdState().alongWith(Commands.idle(this).onlyIf(() -> readyToShoot()).repeatedly());
  }

  /** Hold the current shooting state. */
  public Command holdState() {
    return setFlywheel().andThen(setTurret());
  }

  /** Run magazine */
  public Command runMagazine() {
    return Commands.runOnce(() -> m_Magazine.set(1.0), this);
  }

  /** Stop magazine */
  public Command stopMagazine() {
    return Commands.runOnce(() -> m_Magazine.set(0.0), this);
  }

  /**
   * Checks whether the shooter has reached its target aiming state.
   *
   * @return true when the shooter is aligned and ready to fire.
   */
 public boolean readyToShoot() {
    return m_TurretCANcoder.getAbsolutePosition().isNear(turretSetpoint,kTurretPositionTolerance) &&
    m_Turret.getVelocity().isNear(m_FlywheelOutputDutyCycle,kFlywheelRPMTolerance);
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