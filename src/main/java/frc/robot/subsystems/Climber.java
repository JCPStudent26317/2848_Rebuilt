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
    
    private final TalonFX m_Climber = new TalonFX(kClimberMotorID);
    private final TalonFXConfiguration climberMotorConfig = new TalonFXConfiguration();

    public Climber() {
        setupTalonFx(m_Climber, climberMotorConfig);

        m_Climber.setPosition(0.0);
    }

    public boolean atTop() {
        return m_Climber.getPosition().getValueAsDouble() >= 10;
    }

    public boolean atBottom() {
        return m_Climber.getPosition().getValueAsDouble() <= 0;
    }


    public Command raise() {
        return Commands.run(() -> m_Climber.set(1.0)).until(this::atTop)
        .finallyDo(() -> m_Climber.set(0.0));
    }

    public Command lower() {
        return Commands.run(() -> m_Climber.set(-1.0)).until(this::atBottom)
        .finallyDo(() -> m_Climber.set(0.0));
    }

    public Command stop() {
        return Commands.runOnce(() -> m_Climber.set(0.0));
    }

    public Command directControl(DoubleSupplier input) {
        return Commands.run(() -> m_Climber.set(input.getAsDouble()), this);
    }

}