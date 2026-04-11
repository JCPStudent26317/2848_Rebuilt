package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import lombok.Getter;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.fasterxml.jackson.databind.ser.std.CalendarSerializer;

import static frc.robot.Constants.HopperConstants.*;
import static frc.robot.RangerHelpers.setupTalonFx;

import java.util.function.BooleanSupplier;

public class HopperTransition extends SubsystemBase {
    private final TalonFX m_SidewaysBelt = new TalonFX(kSidewaysBeltMotorID);
    private final TalonFXConfiguration sidewaysBeltMotorConfig = new TalonFXConfiguration();
    private final DutyCycleOut m_SidewaysBeltOut = new DutyCycleOut(0.0);
    
    private final TalonFX m_ForwardBelt = new TalonFX(kForwardBeltMotorID);
    private final TalonFXConfiguration forwardBeltMotorConfig = new TalonFXConfiguration();
    @Getter private final DutyCycleOut m_ForwardBeltOut = new DutyCycleOut(0.0);

    private double forwardBeltSpeed = kForwardBeltSpeed;
    private double sidewaysBeltSpeed = kSidewaysBeltSpeed;

    private boolean unjamming = false;

    public HopperTransition() {
        // Apply things to the configurations here

        setupTalonFx(m_SidewaysBelt, sidewaysBeltMotorConfig);
        setupTalonFx(m_ForwardBelt, forwardBeltMotorConfig);

        this.register();

    }

    @Override
    public void periodic() {
        m_SidewaysBelt.setControl(m_SidewaysBeltOut);
        m_ForwardBelt.setControl(m_ForwardBeltOut);
        
        SmartDashboard.putData(this);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        // builder.addDoubleProperty("Sideways Belt Speed", () -> sidewaysBeltSpeed, (next) -> sidewaysBeltSpeed = next);
        // builder.addDoubleProperty("Forwards Belt Speed", () -> forwardBeltSpeed, (next) -> forwardBeltSpeed = next);

        builder.addBooleanProperty("Is unjamming", () -> unjamming, null);
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

    public Command onlyForwardForward(){
        return Commands.runOnce(()-> setMotorsOutput(m_SidewaysBeltOut.Output,forwardBeltSpeed));
    }
    public Command onlyForwardStop(){
        return Commands.runOnce(()-> setMotorsOutput(m_SidewaysBeltOut.Output,0));
    }
    public Command onlyForwardReverse(){
        return Commands.runOnce(()-> setMotorsOutput(m_SidewaysBeltOut.Output,forwardBeltSpeed *-1));
    }
    public Command onlyForwardSideways(){
        return Commands.runOnce(()-> setMotorsOutput(sidewaysBeltSpeed,m_ForwardBeltOut.Output));
    }
    public Command onlyStopSideways(){
        return Commands.runOnce(()-> setMotorsOutput(0,m_ForwardBeltOut.Output));
    }
    public Command onlyBackwardSideways(){
        return Commands.runOnce(()-> setMotorsOutput(sidewaysBeltSpeed * -1,m_ForwardBeltOut.Output));
    }

    public Command stop() {
        return Commands.runOnce(() -> setMotorsOutput(0.0, 0.0), this);
    }

    /**
     * Runs the forwards belt forwards and sideways belt + star backwards, ends after specific number of seconds
     */
    public Command unjamSideways(double seconds) {
        return new SequentialCommandGroup(
            Commands.runOnce(() -> unjamming = true),
            forward(),
            onlyBackwardSideways().withDeadline(new WaitCommand(seconds)),
            Commands.runOnce(() -> unjamming = false)
        );
    }

    /**
     * Runs both belts backwards, ends after specific number of seconds
     */
    public Command unjamAll(double seconds) {
        return new SequentialCommandGroup(
            Commands.runOnce(() -> unjamming = true),
            backward().withDeadline(new WaitCommand(seconds)),
            Commands.runOnce(() -> unjamming = false)
        );
    }

    /**
     * Note that this command runs repeatedly, while the other commands to set the hopper are instantaneous
     */
    public Command forwardWithAutoUnjam(BooleanSupplier isJammed) {
        return unjamSideways(0.333)
        .andThen(new ConditionalCommand(
            new SequentialCommandGroup(
                unjamSideways(0.4),
                new ConditionalCommand(
                    new SequentialCommandGroup(
                        unjamAll(0.4),
                        forward().withDeadline(new WaitCommand(2.5))
                        ),
                    forward(), isJammed)
            ), 
            forward(),
            isJammed).repeatedly()
            .finallyDo(() -> {setMotorsOutput(0.0, 0.0); unjamming = false;}));
    }

}
