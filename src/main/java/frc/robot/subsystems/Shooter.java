package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ShooterConstants.*;
import static frc.robot.RangerHelpers.*;

import java.util.regex.Matcher;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
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

  private double turretSetpoint = 0;

  private double targetTheta = 0;

  private double distanceTrim = 0;
  private double angularTrim = 0;
  
  private enum flywheelStates {
    IDLE,
    SHOOT
  };

  private flywheelStates currentState = flywheelStates.IDLE;
  // create a Motion Magic request, voltage output


  //private final PositionVoltage m_HoodVoltage = new PositionVoltage(0).withSlot(0);

  //private @Getter double m_FlywheelOutputDutyCycle = 0;
  private @Getter long m_TurretAngle = 0; // Use Radians, 0 is from the front of the robot

  private final VelocityVoltage flyWheelVelocityVoltage = new VelocityVoltage(10);
  private final VelocityVoltage magazineVelocityVoltage = new VelocityVoltage(0);
  private final MotionMagicVoltage turretOut = new MotionMagicVoltage(0);

  /** Shooter Subsystem. */
  public Shooter() {
    m_FlywheelLeftLeader = new TalonFX(kFlywheelLeftMotorID);
    m_FlywheelRightFollower = new TalonFX(kFlywheelRightMotorID);
    m_Turret = new TalonFX(kTurretMotorID);
    m_TurretCANcoder = new CANcoder(kTurretCANcoderID);
    m_Magazine = new TalonFX(kMagazineMotorID);

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
    setupTalonFx(m_FlywheelLeftLeader, flywheelConfig);
    m_FlywheelRightFollower.setControl(new Follower(m_FlywheelLeftLeader.getDeviceID(), MotorAlignmentValue.Aligned));

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
    
    m_Turret.getConfigurator().apply(motionMagicConfigs);

    m_TurretCANcoder.getConfigurator().apply(kTurretCANcoderMagnetSensorConfigs);
    

    TalonFXConfiguration magazineConfig = new TalonFXConfiguration();
    magazineConfig.Slot0.kS = kMagazinekS;
    magazineConfig.Slot0.kV = kMagazinekV;
    magazineConfig.Slot0.kA = kMagazinekA;
    magazineConfig.Slot0.kP = kMagazinekP;  
    magazineConfig.Slot0.kI = kMagazinekI;
    magazineConfig.Slot0.kD = kMagazinekD;
    magazineConfig.Voltage
        .withPeakForwardVoltage(Volts.of(kMagazinePeakVoltage))
        .withPeakReverseVoltage(Volts.of(-1 * kMagazinePeakVoltage));
    magazineConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    retryConfigApply(() -> m_Magazine.getConfigurator().apply(magazineConfig));
  }

  @Override
  public void periodic() {
    SmartDashboard.putData(this);

    targetTheta = (RobotContainer.getDrivetrain().getTargetTheta());
    targetDist = RobotContainer.getDrivetrain().getTargetDist();

    m_Turret.setControl(turretOut.withPosition(MathUtil.clamp(turretSetpoint,kTurretSwitchReverseLimit,kTurretSwitchForwardLimit)).withFeedForward(getTurretFFCorrection()));
    m_FlywheelLeftLeader.setControl(flyWheelVelocityVoltage.withSlot(0));
    m_Magazine.setControl(magazineVelocityVoltage.withSlot(0));
    
    setTurretAngle(targetTheta,true);
  }
  
  private double lastTargetTheta = 0;
  private double lastTimeStamp = Timer.getFPGATimestamp();
  private double getTurretFFCorrection(){
    double omegaFF = MathUtil.angleModulus(targetTheta - lastTargetTheta) / .02;
    lastTargetTheta = targetTheta;
    lastTimeStamp = Timer.getFPGATimestamp();
    return kTurretCorrectionkV * omegaFF + kTurretCorrectionkS * Math.signum(omegaFF);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addDoubleProperty("tangent offset",
    this::getTurretTangentOffset,
    null);

    builder.addDoubleProperty("Feedforward correct",
    this::getTurretFFCorrection,
    null
    );
    builder.addDoubleProperty("Flywheel rps",
      this::getFlywheelRPS,
      null
    );
    builder.addDoubleProperty("distance rps",
    ()->getVeloRPS(getExitVelo()),
    null);
    builder.addDoubleProperty("Commanded flywheel rps",
      ()->flyWheelVelocityVoltage.Velocity
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
    builder.addDoubleProperty("Distance",()->targetDist,
    null);
    builder.addDoubleProperty("flywheel direct output",
    ()->m_FlywheelLeftLeader.get(),
    null);
    builder.addDoubleProperty("flywheel error", 
    ()->m_FlywheelLeftLeader.getClosedLoopError().getValueAsDouble(),
     null);
     builder.addBooleanProperty("ready to shoot",
     this::readyToShoot,
     null);
     builder.addDoubleProperty("magazine output",
     ()->m_Magazine.get(),
     null);
    builder.addDoubleProperty("magazine rps",
    this::getMagazineRPS, null);



    
  } 

/**
 * gets the needed exit velocity of the ball to reach the goal
 * @return velocity in m/s
 */
private double getExitVelo(){
      return RobotContainer.getDrivetrain().getPolarVelocity().getX() * 0.66516439
      + targetDist * 2
      +5.35;
}
/**
 * gets the angle needed to add to the turret setpoint to account for tangential velocity around the target
 * @return the angle needed to add ccw positive, radians
 */
public double getTurretTangentOffset(){
  return 2 * Math.atan2(RobotContainer.getDrivetrain().getPolarVelocity().getY(),getExitVelo()*Math.cos(Math.PI/3));
}
/**
 * turns the needed exit velocity in m/s to rps for motor control, added multiplier to account for slip
 * @param velo
 * @return
 */
private double getVeloRPS(double velo){
  return MathUtil.clamp((velo)/(Math.PI*kFlywheelRadius) * kFlywheelRPMMult,-80,85);
}
/**
 * gets the current turret angle, 0 rad is straight forwards towards the intake [-pi,pi]
 * @return current turret angle, radians
 */
public double getTurretAngle(){
    
    double rawAngle = m_Turret.getPosition().getValueAsDouble() * 2 * Math.PI;
    double shiftedAngle = rawAngle + Math.PI/2;
    double wrapped = MathUtil.angleModulus(shiftedAngle);

    return wrapped;
}

public void trimRight(){
  angularTrim +=.05;
}
public void trimLeft(){
  angularTrim -=.05;
}
public void resetAngularTrim(){
  angularTrim =0;
}

public void trimFurther(){
  distanceTrim +=.1;
}
public void trimCloser(){
  distanceTrim -=.1;
}
public void resetDistanceTrim(){
  distanceTrim =0;
}

/**
 * sets the turret setpoint
 * @param angle in radians [-pi,pi]
 * @param tangentAdjust whether or not to include tangential velocity adjustments
 */
public void setTurretAngle(double angle,boolean tangentAdjust){
  //turretSetpoint = (angle + (tanjentAdjust ? getTurretTangentOffset() : 0)) /(2*Math.PI);
  // 1. Offset for turret encoder zero (pi/2 left)
    double desiredAngle =angle - Math.PI / 2;
    // 2. Wrap math cleanly (optional)
    desiredAngle = MathUtil.angleModulus(desiredAngle + (tangentAdjust ? getTurretTangentOffset() : 0) + angularTrim);
    turretSetpoint = desiredAngle / (2*Math.PI) + angularTrim;
}

/**
 * gets flywheel rps
 * @return velocity of flywheel in rps
 */
  public double getFlywheelRPS(){
    return m_FlywheelLeftLeader.getVelocity().getValueAsDouble();
  }

  public Command idleFlywheel(){
    //return new InstantCommand(()->m_FlywheelOutputDutyCycle = kFlywheelIdleSpeed,this);
    return Commands.runOnce(() -> flyWheelVelocityVoltage.Velocity = kFlywheelIdleSpeed, this);
  }
  public Command runFlywheel(){
    return Commands.runOnce(() -> flyWheelVelocityVoltage.Velocity = getVeloRPS(getExitVelo())+distanceTrim, this);
  }

  /** Sets the flywheel and hood angle to their shot velocity and shot position. */
  public Command shoot() {
      //return runFlywheel().repeatedly().andThen(runMagazine().onlyIf(()->readyToShoot()));
      return runFlywheel().repeatedly().until(()->readyToShoot()).andThen(runMagazine()).andThen(runFlywheel().repeatedly());
  }


  /** Run magazine */
  public Command runMagazine() {
    return Commands.runOnce(() -> magazineVelocityVoltage.Velocity = 75, this);
  }

  /** Stop magazine */
  public Command stopMagazine() {
    return Commands.runOnce(() -> magazineVelocityVoltage.Velocity = 0.0, this);
  }
  
/**
 * gets flywheel rps
 * @return velocity of flywheel in rps
 */
  public double getMagazineRPS(){
    return m_Magazine.getVelocity().getValueAsDouble();
  }  

  /**
   * Checks whether the shooter has reached its target aiming state.
   *
   * @return true when the shooter is aligned and ready to fire.
   */
 public boolean readyToShoot() {
    return m_TurretCANcoder.getAbsolutePosition().isNear(turretSetpoint,kTurretPositionTolerance) &&
    m_FlywheelLeftLeader.getVelocity().isNear(flyWheelVelocityVoltage.Velocity ,kFlywheelRPSTolerance);
  }
}