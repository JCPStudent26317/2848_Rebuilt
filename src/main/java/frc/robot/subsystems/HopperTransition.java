package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import static frc.robot.Constants.HopperConstants.*;
import static frc.robot.RangerHelpers.setupTalonFx;

public class HopperTransition extends SubsystemBase {
    private final TalonFX m_SidewaysBelt = new TalonFX(kSidewaysBeltMotorID);
    private final TalonFXConfiguration sidewaysBeltMotorConfig = new TalonFXConfiguration();
    private final DutyCycleOut m_SidewaysBeltOut = new DutyCycleOut(0.0);
    
    private final TalonFX m_ForwardBelt = new TalonFX(kForwardBeltMotorID);
    private final TalonFXConfiguration forwardBeltMotorConfig = new TalonFXConfiguration();
    @Getter private final DutyCycleOut m_ForwardBeltOut = new DutyCycleOut(0.0);


    public HopperTransition() {
        // Apply things to the configurations here

        setupTalonFx(m_SidewaysBelt, sidewaysBeltMotorConfig);
        setupTalonFx(m_ForwardBelt, forwardBeltMotorConfig);

    }

    public Command holdState() {
        // setControl needs to run periodically at all times or the motor will disable I think
        return Commands.run(() -> {
            m_SidewaysBelt.setControl(m_SidewaysBeltOut);
            m_ForwardBelt.setControl(m_ForwardBeltOut);
        },this);
    }    


    @Override
    public void periodic(){ 
    }

    public void setMotorsOutput(double sidewaysBeltOutput, double kForwardBeltOutput) {
        m_SidewaysBeltOut.Output = sidewaysBeltOutput;
        m_ForwardBeltOut.Output = kForwardBeltOutput;
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
