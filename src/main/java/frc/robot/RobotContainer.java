// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.HopperTransition;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import lombok.Getter;
import lombok.Setter;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverJoystick = new CommandXboxController(0);
    private final CommandXboxController testingJoystick = new CommandXboxController(5);

    @Getter public static final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    @Getter public static final Vision vision = new Vision();
    public static final Intake intake = new Intake();
    public static final HopperTransition hopper = new HopperTransition();
    public static final Magazine magazine = new Magazine();
    @Getter public static final Shooter shooter = new Shooter();


    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {

        shooter.register();
        magazine.register();
        hopper.register();
        intake.register();

        intake.setDefaultCommand(intake.holdState());
        hopper.setDefaultCommand(hopper.holdState());



        driverJoystick.a().onTrue(new InstantCommand(()->shooter.flyWheelOn()));
        driverJoystick.a().onFalse(new InstantCommand(()->shooter.flyWheelOff()));

        driverJoystick.b().onTrue(magazine.run());
        driverJoystick.b().onFalse(magazine.stop());
        driverJoystick.b().onTrue(hopper.forward());
        driverJoystick.b().onFalse(hopper.stop());

        driverJoystick.x().onTrue(intake.intake());
        driverJoystick.x().onFalse(intake.stop());

        //driverJoystick.y().onTrue(new InstantCommand(()->hopper.hopperBackwards()));

        driverJoystick.y().onTrue(shooter.setTurret());
        


        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        // drivetrain.setDefaultCommand(
        //     // Drivetrain will execute this command periodically
        //     drivetrain.applyRequest(() ->
        //         drive.withVelocityX(-driverJoystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
        //             .withVelocityY(-driverJoystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
        //             .withRotationalRate(-driverJoystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        //     )
        // );


        driverJoystick.leftBumper().onTrue(new InstantCommand(()->drivetrain.visionOdoReset()));

        // driverJoystick.rightBumper().whileTrue(drivetrain.applyRequest(() ->
        //         drive.withVelocityX(-driverJoystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
        //             .withVelocityY(-driverJoystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
        //             .withRotationalRate(-drivetrain.getPIDTurn()) // Drive counterclockwise with negative X (left)
        //     ));

        /*
        driverJoystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driverJoystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driverJoystick.getLeftY(), -driverJoystick.getLeftX()))
        ));
        */

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        /*
        driverJoystick.back().and(driverJoystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverJoystick.back().and(driverJoystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverJoystick.start().and(driverJoystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverJoystick.start().and(driverJoystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        */

        driverJoystick.pov(0).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverJoystick.pov(90).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverJoystick.pov(180).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverJoystick.pov(270).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on back press
        driverJoystick.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        // driverJoystick.a().onTrue(intake.intake());
        // driverJoystick.a().onFalse(intake.stop());

        // driverJoystick.b().onTrue(hopper.forward());
        // driverJoystick.b().onFalse(hopper.stop());

        // driverJoystick.x().onTrue(magazine.run());
        // driverJoystick.x().onFalse(magazine.stop());
    }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
