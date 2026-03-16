package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.fasterxml.jackson.databind.ser.std.CalendarSerializer;

import static frc.robot.Constants.HopperConstants.*;
import static frc.robot.RangerHelpers.setupTalonFx;

public class HopperTransition extends SubsystemBase {
    private final TalonFX m_SidewaysBelt = new TalonFX(kSidewaysBeltMotorID);
    private final TalonFXConfiguration sidewaysBeltMotorConfig = new TalonFXConfiguration();
    private final DutyCycleOut m_SidewaysBeltOut = new DutyCycleOut(0.0);

    private final CANrange m_MagazineSensor = new CANrange(kCANRangeID);
    private final CANrangeConfiguration magazineSensorConfig = new CANrangeConfiguration();
    
    private final TalonFX m_ForwardBelt = new TalonFX(kForwardBeltMotorID);
    private final TalonFXConfiguration forwardBeltMotorConfig = new TalonFXConfiguration();
    @Getter private final DutyCycleOut m_ForwardBeltOut = new DutyCycleOut(0.0);

    private double forwardBeltSpeed = kForwardBeltSpeed;
    private double sidewaysBeltSpeed = kSidewaysBeltSpeed;

    public HopperTransition() {
        // Apply things to the configurations here

        magazineSensorConfig.ProximityParams.ProximityThreshold =.085;

        m_MagazineSensor.getConfigurator().apply(magazineSensorConfig);

        setupTalonFx(m_SidewaysBelt, sidewaysBeltMotorConfig);
        setupTalonFx(m_ForwardBelt, forwardBeltMotorConfig);

        this.register();

    }

    @Override
    public void periodic() {
        m_SidewaysBelt.setControl(m_SidewaysBeltOut);
        m_ForwardBelt.setControl(m_ForwardBeltOut);

        SmartDashboard.putNumber("CANRange detection",m_MagazineSensor.getIsDetected().getValueAsDouble());
        
        SmartDashboard.putData(this);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addDoubleProperty("Sideways Belt Speed", () -> sidewaysBeltSpeed, (next) -> sidewaysBeltSpeed = next);
        builder.addDoubleProperty("Forwards Belt Speed", () -> forwardBeltSpeed, (next) -> forwardBeltSpeed = next);
    }

    public Command holdState() {
        return Commands.idle(this);
    }    

    public void setMotorsOutput(double sidewaysBeltOutput, double kForwardBeltOutput) {
        m_SidewaysBeltOut.Output = sidewaysBeltOutput;
        m_ForwardBeltOut.Output = kForwardBeltOutput;
    }

    public Command forward() {
        return Commands.runOnce(() -> setMotorsOutput(sidewaysBeltSpeed, forwardBeltSpeed), this);
    }

    public Command backward() {
        return Commands.runOnce(() -> setMotorsOutput(sidewaysBeltSpeed * -1, forwardBeltSpeed * -1), this);
    }

    public Command stop() {
        return Commands.runOnce(() -> setMotorsOutput(0.0, 0.0), this);
    }
    
    public Command jiggle(){
        return new SequentialCommandGroup(
        forward()
        ,Commands.waitSeconds(.5)
        ,backward()
        ,Commands.waitSeconds(.25)
        ).repeatedly();

    }

}
