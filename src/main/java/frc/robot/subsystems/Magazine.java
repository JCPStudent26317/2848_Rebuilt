package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import static frc.robot.Constants.MagazineConstants.*;

public class Magazine extends SubsystemBase {

    private final TalonFX magazineMotor = new TalonFX(kMagazineMotorID);
    private final TalonFXConfiguration magazineMotorConfig = new TalonFXConfiguration();
    private final DutyCycleOut magazineOut = new DutyCycleOut(0.0);    

    public Magazine() {
        // Apply motor configurations here

        magazineMotor.getConfigurator().apply(magazineMotorConfig);
    }

    public void periodic(){
        magazineMotor.setControl(magazineOut);
        SmartDashboard.putNumber("Magazine RPM", getMagazineRPM());
    }

    public double getMagazineRPM(){
        return magazineMotor.getVelocity().getValueAsDouble()*60;
    }


    public Command holdState() {
        // setControl needs to run periodically at all times or the motor will disable I think
        return Commands.run(() -> magazineMotor.setControl(magazineOut));
    }

    public void setMotorOutput(double output) {
        magazineOut.Output = output*-1;
    }
    
    public Command run() {
        return Commands.runOnce(() -> setMotorOutput(kMagazineMotorSpeed));
    }

    public Command stop() {
        return Commands.runOnce(() -> setMotorOutput(0.0));
    }

}
