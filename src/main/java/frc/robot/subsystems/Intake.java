package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import static frc.robot.Constants.IntakeConstants.*;

public class Intake extends SubsystemBase {
    private final TalonFX intakeMotor = new TalonFX(kIntakeMotorID);
    private final TalonFXConfiguration kIntakeMotorConfig = new TalonFXConfiguration();

    public Intake() {
        // Configuration here
    }

    public Command holdState() {
        return Commands.idle();
    }

}
