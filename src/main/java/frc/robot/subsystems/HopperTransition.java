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
import static frc.robot.RangerHelpers.setupTalonFx;

public class HopperTransition extends SubsystemBase {
    private final TalonFX sidewaysBeltMotor = new TalonFX(kSidewaysBeltMotorID);
    private final TalonFXConfiguration sidewaysBeltMotorConfig = new TalonFXConfiguration();
    private final DutyCycleOut sidewaysBeltOut = new DutyCycleOut(0.0);
    
    private final TalonFX forwardBeltMotor = new TalonFX(kForwardBeltMotorID);
    private final TalonFXConfiguration forwardBeltMotorConfig = new TalonFXConfiguration();
    @Getter private final DutyCycleOut forwardBeltOut = new DutyCycleOut(0.0);

    private final Timer timer = new Timer();

    public HopperTransition() {
        // Apply things to the configurations here

        setupTalonFx(sidewaysBeltMotor, sidewaysBeltMotorConfig);
        setupTalonFx(forwardBeltMotor, forwardBeltMotorConfig);

        timer.reset();
        timer.start();
    }

    public Command holdState() {
        // setControl needs to run periodically at all times or the motor will disable I think
        return Commands.run(() -> {
            sidewaysBeltMotor.setControl(sidewaysBeltOut);
            forwardBeltMotor.setControl(forwardBeltOut);
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

    public void setMotorsOutput(double sidewaysBeltOutput, double kForwardBeltOutput) {
        sidewaysBeltOut.Output = sidewaysBeltOutput;
        forwardBeltOut.Output = kForwardBeltOutput;
    }

    public Command forward() {
        return Commands.runOnce(() -> setMotorsOutput(kSidewaysBeltSpeed, kForwardBeltSpeed));
    }

    public Command backward() {
        return Commands.runOnce(() -> setMotorsOutput(kSidewaysBeltSpeed * -1, kForwardBeltSpeed * -1));
    }

    public Command stop() {
        return Commands.runOnce(() -> setMotorsOutput(0.0, 0.0));
    }

}
