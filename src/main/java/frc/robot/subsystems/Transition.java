package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import static frc.robot.Constants.TransitionConstants.*;

public class Transition extends SubsystemBase {
    private final TalonFX transitionMotor = new TalonFX(kTransitionMotorID);
    private final TalonFXConfiguration transitionMotorConfig = new TalonFXConfiguration();
    private final DutyCycleOut transitionOut = new DutyCycleOut(0.0);

    public Transition() {
        // Apply motor configurations here

        transitionMotor.getConfigurator().apply(transitionMotorConfig);
    }

    public Command holdState() {
        // setControl needs to run periodically at all times or the motor will disable I think
        return Commands.run(() -> transitionMotor.setControl(transitionOut));
    }

    public void setMotorOutput(double output) {
        transitionOut.Output = output;
    }
    
    public Command run() {
        return Commands.runOnce(() -> setMotorOutput(kTransitionMotorSpeed));
    }

    public Command stop() {
        return Commands.runOnce(() -> setMotorOutput(0.0));
    }

}
