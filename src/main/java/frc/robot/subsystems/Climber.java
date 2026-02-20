package frc.robot.subsystems;

import static frc.robot.Constants.ClimberConstants.*;
import static frc.robot.RangerHelpers.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    
    private final TalonFX climberMotor = new TalonFX(kClimberMotorID);
    private final TalonFXConfiguration climberMotorConfig = new TalonFXConfiguration();

    public Climber() {
        setupTalonFx(climberMotor, climberMotorConfig);
    }

}