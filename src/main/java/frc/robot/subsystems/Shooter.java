package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ShooterConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.StatusCode;
import frc.robot.RangerHelpers;
import lombok.Getter;

/** Shooter Subsystem. */
public class Shooter extends SubsystemBase{
  private final TalonFX m_Flywheel;

  private final TalonFX m_Turret;

  private final TalonFX m_Hood;

  private final VelocityVoltage m_FlywheelVV = new VelocityVoltage(0).withSlot(0);
  private final DutyCycleOut m_FlywheelOut = new DutyCycleOut(0);

  private final PositionVoltage m_TurretPV = new PositionVoltage(0).withSlot(0);

  private final PositionVoltage m_HoodVoltage = new PositionVoltage(0).withSlot(0);

  public Shooter(){
    m_Flywheel = new TalonFX(kFlywheelMotorID);
    m_Turret = new TalonFX(kTurretMotorID);
    m_Hood = new TalonFX(kHoodMotorID);

    TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
    flywheelConfig.Slot0.kS = 0;
    flywheelConfig.Slot0.kV = 0;
    flywheelConfig.Slot0.kP = 0;  
    flywheelConfig.Slot0.kI = 0;
    flywheelConfig.Slot0.kD = 0;
    flywheelConfig.Voltage.withPeakForwardVoltage(Volts.of(8)).withPeakReverseVoltage(Volts.of(-8));
    RangerHelpers.setupTalonFX(m_Flywheel, flywheelConfig);


    TalonFXConfiguration turretConfig = new TalonFXConfiguration();
    turretConfig.Slot0.kS = 0;
    turretConfig.Slot0.kV = 0;
    turretConfig.Slot0.kP = 0;
    turretConfig.Slot0.kI = 0;
    turretConfig.Slot0.kD = 0;
    turretConfig.Voltage
        .withPeakForwardVoltage(Volts.of(6))
        .withPeakReverseVoltage(Volts.of(-6));
    RangerHelpers.setupTalonFX(m_Turret, turretConfig);


    TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
    hoodConfig.Slot0.kS = 0;
    hoodConfig.Slot0.kV = 0;
    hoodConfig.Slot0.kP = 0;
    hoodConfig.Slot0.kI = 0;
    hoodConfig.Slot0.kD = 0;
    hoodConfig.Voltage
        .withPeakForwardVoltage(Volts.of(6))
        .withPeakReverseVoltage(Volts.of(-6));
    RangerHelpers.setupTalonFX(m_Hood, hoodConfig);



  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private void setTurret(){


  }

  private void setHood(){


  }

  private void setFlywheel(int vel){


  }

  public void setTarget(){


  }

  public Command shoot(){
    return holdState().alongWith(Commands.idle(this).onlyIf(() -> readyToShoot()).repeatedly());
  }

  public Command holdState(){
    return null;
  }

  public boolean readyToShoot(){
    return false;
  }

}
