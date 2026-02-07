package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import static frc.robot.Constants.HopperConstants.*;

public class HopperTransition extends SubsystemBase {
    private final TalonFX beltMotor = new TalonFX(kBeltMotorID);
    private final TalonFXConfiguration beltMotorConfig = new TalonFXConfiguration();
    private final DutyCycleOut beltOut = new DutyCycleOut(0.0);
    
    private final TalonFX rollerMotor = new TalonFX(kRollerMotorID);
    private final TalonFXConfiguration rollerMotorConfig = new TalonFXConfiguration();
    @Getter private final DutyCycleOut rollerOut = new DutyCycleOut(0.0);

    private final TalonFX transitionMotor = new TalonFX(kTransitionMotorID);
    private final TalonFXConfiguration transitionMotorConfig = new TalonFXConfiguration();
    private final DutyCycleOut transitionOut = new DutyCycleOut(0.0);
    private final Timer timer = new Timer();

    public HopperTransition() {
        // Apply things to the configurations here

        transitionMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        beltMotor.getConfigurator().apply(beltMotorConfig);
        rollerMotor.getConfigurator().apply(rollerMotorConfig);
        transitionMotor.getConfigurator().apply(transitionMotorConfig);

        timer.reset();
        timer.start();
    }

    public Command holdState() {
        // setControl needs to run periodically at all times or the motor will disable I think
        return Commands.run(() -> {
            beltMotor.setControl(beltOut);
            rollerMotor.setControl(rollerOut);
            transitionMotor.setControl(rollerOut);
        },this);
    }    

    private boolean nextBack = false;

    @Override
    public void periodic(){
        // if(timer.get()>1 &&timer.get()<1.25&& !nextBack){
        //     nextBack = true;
        // } else if(timer.get()>1.25 &&nextBack){
        //     rollerOut.Output = -rollerOut.Output;
        //     timer.reset();
        //     timer.start();
        //     nextBack = false;
        // }
        // SmartDashboard.putNumber("Roller output",getRollerOut().Output);

        
    }

    public void hopperBackwards(){
        beltOut.Output = -.66;
    }
    public void hopperForwards(){
        beltOut.Output = .66;
    }

    public void setMotorsOutput(double beltOutput, double rollerOutput, double transitionOutput) {
        beltOut.Output = beltOutput;
        rollerOut.Output = rollerOutput;
        transitionOut.Output = transitionOutput;
    }

    public Command forward() {
        return Commands.runOnce(() -> setMotorsOutput(kBeltMotorSpeed, kRollerMotorSpeed, kTransitionMotorSpeed));
    }

    public Command backward() {
        return Commands.runOnce(() -> setMotorsOutput(kBeltMotorSpeed * -1, kRollerMotorSpeed * -1, kTransitionMotorSpeed * -1));
    }

    public Command stop() {
        return Commands.runOnce(() -> setMotorsOutput(0.0, 0.0, 0.0));
    }

}
