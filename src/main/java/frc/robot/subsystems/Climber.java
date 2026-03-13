package frc.robot.subsystems;

import static frc.robot.Constants.ClimberConstants.*;
import static frc.robot.RangerHelpers.*;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    
    private final TalonFX m_Climber = new TalonFX(kClimberMotorID);
    private final TalonFXConfiguration climberMotorConfig = new TalonFXConfiguration();

    private final CANdi limitSwitch = new CANdi(kLimitSwitchID);

    public Climber() {

        climberMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        setupTalonFx(m_Climber, climberMotorConfig);

        m_Climber.setPosition(0.0);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addBooleanProperty("Limit Switch S1Closed", () -> limitSwitch.getS1Closed().getValue(), null); // At bottom when S1 true
        builder.addBooleanProperty("Limit Switch S2Closed", () -> limitSwitch.getS2Closed().getValue(), null);
    }

    @Override
    public void periodic() {
        SmartDashboard.putData(this);
    }

    public boolean atTop() {
        return m_Climber.getPosition().getValueAsDouble() >= 10;
    }

    public boolean atBottom() {
        return m_Climber.getPosition().getValueAsDouble() <= 0;
    }

    public Command raise() {
        return Commands.run(() -> m_Climber.set(1.0), this).until(this::atTop)
        .finallyDo(() -> m_Climber.set(0.0));
    }

    public Command lower() {
        return Commands.run(() -> m_Climber.set(-1.0), this).until(this::atBottom)
        .finallyDo(() -> m_Climber.set(0.0));
    }

    public Command stop() {
        return Commands.runOnce(() -> m_Climber.set(0.0), this);
    }

    public Command directControl(DoubleSupplier input) {
        return Commands.run(() -> m_Climber.set(
            Math.abs(input.getAsDouble()) < 0.066 ? 0 : input.getAsDouble()), this);
    }

}