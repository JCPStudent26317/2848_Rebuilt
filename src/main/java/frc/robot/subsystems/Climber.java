package frc.robot.subsystems;

import static frc.robot.Constants.ClimberConstants.*;
import static frc.robot.RangerHelpers.*;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    
    private final TalonFX climberMotor = new TalonFX(kClimberMotorID);
    private final TalonFXConfiguration climberMotorConfig = new TalonFXConfiguration();

    public Climber() {
        setupTalonFx(climberMotor, climberMotorConfig);

        climberMotor.setPosition(0.0);
    }

    public boolean atTop() {
        return climberMotor.getPosition().getValueAsDouble() >= 10;
    }

    public boolean atBottom() {
        return climberMotor.getPosition().getValueAsDouble() <= 0;
    }


    public Command raise() {
        return Commands.run(() -> climberMotor.set(1.0)).until(this::atTop)
        .finallyDo(() -> climberMotor.set(0.0));
    }

    public Command lower() {
        return Commands.run(() -> climberMotor.set(-1.0)).until(this::atBottom)
        .finallyDo(() -> climberMotor.set(0.0));
    }

    public Command stop() {
        return Commands.runOnce(() -> climberMotor.set(0.0));
    }

    public Command directControl(DoubleSupplier input) {
        return Commands.run(() -> climberMotor.set(input.getAsDouble()), this);
    }

}