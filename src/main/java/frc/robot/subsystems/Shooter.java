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

  private double targetTheta = 0;

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
    //turretConfig.Feedback.SensorToMechanismRatio =-1;
    turretConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    var motionMagicConfigs = turretConfig.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = kTurretMMCruiseVelocity; 
    motionMagicConfigs.MotionMagicAcceleration = kTurretMMAcceleration; 
    motionMagicConfigs.MotionMagicJerk = kTurretMMJerk;
    //CANcoderConfigurator turretCANcoderConfigurator = m_TurretCANcoder.getConfigurator();
    m_TurretCANcoder.getConfigurator().apply(kTurretCANcoderMagnetSensorConfigs);
    //retryConfigApply(() -> turretCANcoderConfigurator.apply(kTurretCANcoderMagnetSensorConfigs)); 
    setupTalonFx(m_Turret, turretConfig);


    SoftwareLimitSwitchConfigs turretSwitchConfigs = new SoftwareLimitSwitchConfigs();
    turretSwitchConfigs.ForwardSoftLimitThreshold = kTurretSwitchForwardLimit;
    turretSwitchConfigs.ReverseSoftLimitThreshold = kTurretSwitchReverseLimit;
    turretSwitchConfigs.ForwardSoftLimitEnable = true;
    turretSwitchConfigs.ReverseSoftLimitEnable = true;
    m_Turret.getConfigurator().apply(turretSwitchConfigs);
    
    //m_Turret.getConfigurator().apply(motionMagicConfigs);
    

    TalonFXConfiguration magazineConfig = new TalonFXConfiguration();
    magazineConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    retryConfigApply(() -> m_Magazine.getConfigurator().apply(magazineConfig));

    init();
  }

  @Override
  public void periodic() {
    //turretSetpoint = MathUtil.clamp(SmartDashboard.getNumber("turret setpoint",0),kTurretSwitchReverseLimit,kTurretSwitchForwardLimit);
    SmartDashboard.putData(this);

    //turretSetpoint =0;
    targetTheta = (RobotContainer.getDrivetrain().getTargetTheta());
    targetDist = RobotContainer.getDrivetrain().getTargetDist();

    m_FlywheelOutputDutyCycle = getVeloRPM(getExitVelo());

    
    setTurretAngle(targetTheta,false);
  }

  private double lastTargetTheta = 0;
  private double getTurretFFCorrection(){
    double omegaFF = (targetTheta - lastTargetTheta) / .02;
    lastTargetTheta = targetTheta;
    return kTurretCorrectionkV * omegaFF + kTurretCorrectionkS * Math.signum(omegaFF);
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
    builder.addDoubleProperty("Feedforward correct",
    this::getTurretFFCorrection,
    null
    );
    builder.addDoubleProperty("Flywheel rpm",
      this::getFlywheelRPM,
      null
    );
    builder.addDoubleProperty("Commanded flywheel rpm",
      ()->m_FlywheelOutputDutyCycle
      ,null);
    builder.addDoubleProperty("turret angle",
      this::getTurretAngle,
      null);
    builder.addDoubleProperty("Turret Encoder Output",
      ()->m_TurretCANcoder.getAbsolutePosition().getValueAsDouble(),
      null);
    builder.addDoubleProperty("turret error",
      ()->m_Turret.getClosedLoopError().getValueAsDouble(),
      null);
    builder.addDoubleProperty("turret setpoint",
      ()->turretSetpoint,
      null);
    builder.addDoubleProperty("Shooter hub theta",
      ()-> targetTheta,
      null);



    
  } 

  /** Sets the turret angle to aim the shooter at the target.*/
  public Command setTurret() {
    return Commands.runOnce(
      () -> m_Turret.setControl(turretOut.withPosition(turretSetpoint).withFeedForward(getTurretFFCorrection())), 
      this);
  }


public void init(){
  //CommandScheduler.getInstance().schedule(setFlywheel());
  SmartDashboard.putNumber("turret setpoint",turretSetpoint);
  SmartDashboard.putNumber("Distance",targetDist);
  this.register();
}

/**
 * gets the needed exit velocity of the ball to reach the goal
 * @return velocity in m/s
 */
private double getExitVelo(){
      return RobotContainer.getDrivetrain().getPolarVelocity().getX() * 0.66516439
      + targetDist * 0.71612605
      +5.3496863293326635;
}
/**
 * gets the angle needed to add to the turret setpoint to account for tangential velocity around the target
 * @return the angle needed to add ccw positive, radians
 */
public double getTurretTangentOffset(){
  return Math.atan2(RobotContainer.getDrivetrain().getPolarVelocity().getY(),getExitVelo()*Math.cos(Math.PI/3))/(2*Math.PI);
}
/**
 * turns the needed exit velocity in m/s to rpm for motor control, added multiplier to account for slip
 * @param velo
 * @return
 */
private double getVeloRPM(double velo){
  return (velo*60)/(2*Math.PI*kFlywheelRadius) * kFlywheelRPMMult;
}
/**
 * gets the current turret angle, 0 rad is straight forwards towards the intake [-pi,pi]
 * @return current turret angle, radians
 */
public double getTurretAngle(){
  return m_Turret.getPosition().getValueAsDouble() * Math.PI *2;
}

/**
 * sets the turret setpoint
 * @param angle in radians
 * @param tanjentAdjust whether or not to include tangential velocity adjustments
 */
public void setTurretAngle(double angle,boolean tanjentAdjust){
  turretSetpoint = (angle + (tanjentAdjust ? getTurretTangentOffset() : 0)) /(2*Math.PI);
}

/**
 * gets flywheel rpm
 * @return velocity of flywheel in rpm
 */
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
    return Commands.run(()->{CommandScheduler.getInstance().schedule(setFlywheel());
    CommandScheduler.getInstance().schedule(setTurret());},this);
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