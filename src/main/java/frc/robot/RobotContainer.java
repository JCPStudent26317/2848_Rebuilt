// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.security.spec.NamedParameterSpec;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.EventMarker;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
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
    private final SwerveRequest.RobotCentric preciseAdjustments = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        
    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverJoystick = new CommandXboxController(0);
    private final CommandGenericHID keypad = new CommandGenericHID(1);



    @Getter public static final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    @Getter public static final Vision vision = new Vision();
    @Getter public static final Intake intake = new Intake();
    @Getter public static final HopperTransition hopper = new HopperTransition();
    @Getter public static final Shooter shooter = new Shooter();
    // public static final Climber climber = new Climber();
    @Getter public static final Lights lights = new Lights();

    private final BooleanSupplier manualDrivebase = () -> Math.hypot(driverJoystick.getLeftX(), driverJoystick.getLeftY()) > 0.15
                                                                || Math.abs(driverJoystick.getRightX()) > 0.15;

    private final SendableChooser<Command> autoChooser;

    private final Trigger distanceTrigger = new Trigger(()->drivetrain.outOfRange());

    private final SendableChooser<Double> intakeChooser = new SendableChooser<>();;

    private final Trigger readyToShoot = new Trigger(()->shooter.readyToShoot());

    private final Command startShoot = shooter.shoot();
    
        
    //hopper.forward().onlyIf(()->shooter.readyToShoot()).repeatedly()
    private final Command startShootAuto = shooter.shoot().alongWith(
        Commands.waitUntil(()->shooter.readyToShoot()).andThen(            
            hopper.forwardWithAutoUnjam(() -> shooter.isJammed())
        )
    );

    @Getter private final Command stopShoot = shooter.idleFlywheel()
    .andThen(shooter.stopMagazine())
    .andThen(hopper.stop());

    public RobotContainer() {
        NamedCommands.registerCommand("Intake Deploy", intake.deploy());
        NamedCommands.registerCommand("Intake Low Retract", intake.lowRetract());
        NamedCommands.registerCommand("Intake Stow", intake.lowRetract());
        NamedCommands.registerCommand("Intake Run Rollers", intake.intake());
        NamedCommands.registerCommand("Intake Stop Rollers", intake.stop());
        NamedCommands.registerCommand("Intake Jiggle", intake.jiggle());
        
        NamedCommands.registerCommand("Transition Run Belts", hopper.forward());
        NamedCommands.registerCommand("Transition Stop Belts", hopper.stop());
        NamedCommands.registerCommand("Transition Reverse Belts", hopper.backward());

        NamedCommands.registerCommand("Start Shoot",startShootAuto);
        NamedCommands.registerCommand("Stop Shoot",stopShoot);
        

    
        
        NamedCommands.registerCommand("Climb Auto Align", drivetrain.autoAlignClimb());

        autoChooser = AutoBuilder.buildAutoChooser();
        autoChooser.addOption("NeutralPastLine (Left Side)",
            new PathPlannerAuto("NeutralPastLine (Right Side)", true));
        autoChooser.addOption("NeutralWithinLine (Left Side)",
            new PathPlannerAuto("NeutralWithinLine (Right Side)", true));            
        autoChooser.addOption("NeutralDiagonal (Left Side)",
            new PathPlannerAuto("NeutralDiagonal (Right Side)", true));
        autoChooser.addOption("TrenchDoubleNeutral (Left Side)",
            new PathPlannerAuto("TrenchDoubleNeutral (Right Side)", true));            
        autoChooser.addOption("TrenchBumpDoubleNeutral (Left Side)",
            new PathPlannerAuto("TrenchBumpDoubleNeutral (Right Side)", true));            
        
        intakeChooser.setDefaultOption("Deployed",Constants.IntakeConstants.kDeploySetpoint);
        intakeChooser.addOption("Stowed",Constants.IntakeConstants.kStowSetpoint);


        SmartDashboard.putData("Intake Position Chooser",intakeChooser);


        SmartDashboard.putData("Auto Chooser", autoChooser);

        

        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    }

    private void configureBindings() {

        distanceTrigger.and(()->DriverStation.isTeleop() && DriverStation.isEnabled()).onTrue(Commands.runOnce(()->driverJoystick.setRumble(RumbleType.kBothRumble, 0.95)));
        distanceTrigger.onFalse(Commands.runOnce(()->driverJoystick.setRumble(RumbleType.kBothRumble,0)));

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverJoystick.getLeftY() * MaxSpeed / (driverJoystick.rightBumper().getAsBoolean() ? 2.0 : 1.0)) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverJoystick.getLeftX() * MaxSpeed / (driverJoystick.rightBumper().getAsBoolean() ? 2.0 : 1.0)) // Drive left with negative X (left)
                    .withRotationalRate(-driverJoystick.getRightX() * MaxAngularRate / (driverJoystick.rightBumper().getAsBoolean() ? 3.5 : 1.0))
                     // Drive counterclockwise with negative X (left)
                    //.withCenterOfRotation(new Translation2d(.2,0)) // move the center of rotation forward so that when the expanded hopper is deployed the center of rotation is the new center of the rectangular bot.
            )
        );

        

        // driverJoystick.rightBumper().onTrue(Commands.runOnce(()->drivetrain.setSlowDownFactor(5)));
        // driverJoystick.rightBumper().onFalse(Commands.runOnce(()->drivetrain.setSlowDownFactor(1)));

        driverJoystick.rightBumper().onTrue(hopper.forwardWithAutoUnjam(() -> shooter.isJammed()).onlyIf(()->shooter.readyToShoot()).repeatedly());
        driverJoystick.rightBumper().onFalse(hopper.stop());

        driverJoystick.rightBumper().onTrue(Commands.runOnce(()->shooter.setShooting(true)));
        driverJoystick.rightBumper().onFalse(Commands.runOnce(()->shooter.setShooting(false)));


    //     driverJoystick.rightTrigger(Constants.OperatorConstants.kTriggerThreshhold).whileTrue(shooter.shoot()
    // .beforeStarting(()->drivetrain.setTarget(false)).repeatedly()
    //     .beforeStarting(hopper.noJamRun()).onlyIf(()->shooter.readyToShoot()).repeatedly());

        //shooter.setDefaultCommand(shooter.holdState());

        // driverJoystick.rightTrigger(Constants.OperatorConstants.kTriggerThreshhold).onTrue(hopper.jiggle());
        // driverJoystick.rightTrigger(Constants.OperatorConstants.kTriggerThreshhold).onFalse(hopper.stop().onlyIf(()->!driverJoystick.rightBumper().getAsBoolean()));
        // driverJoystick.rightTrigger(Constants.OperatorConstants.kTriggerThreshhold).onFalse(hopper.forward().onlyIf(()->driverJoystick.rightBumper().getAsBoolean()));

        // Small adjustments code
        driverJoystick.pov(90)
                .whileTrue(drivetrain.applyRequest(() -> preciseAdjustments.withVelocityX(0).withVelocityY(-0.2))); //right
        driverJoystick.pov(270)
                .whileTrue(drivetrain.applyRequest(() -> preciseAdjustments.withVelocityX(0).withVelocityY(0.2))); //left
        driverJoystick.pov(0)
                .whileTrue(drivetrain.applyRequest(() -> preciseAdjustments.withVelocityX(0.2).withVelocityY(0)));
        driverJoystick.pov(180)
                .whileTrue(drivetrain.applyRequest(() -> preciseAdjustments.withVelocityX(-0.2).withVelocityY(0)));

        //INTAKE CONTROLS
        driverJoystick.leftTrigger(Constants.OperatorConstants.kTriggerThreshhold).whileTrue(intake.jiggle());
        driverJoystick.leftTrigger(Constants.OperatorConstants.kTriggerThreshhold).onFalse(intake.deploy());

        driverJoystick.leftBumper().onTrue(intake.intake());
        driverJoystick.leftBumper().onFalse(intake.stop());

        driverJoystick.x().onTrue(intake.deploy());
        driverJoystick.b().onTrue(intake.stow());

        //SHOOTER CONTROLS

        driverJoystick.rightBumper().onTrue(startShoot);
        //.alongWith(intake.jiggle().onlyIf(()->!driverJoystick.leftBumper().getAsBoolean()).repeatedly()));

        driverJoystick.rightBumper().onFalse(stopShoot);
        //driverJoystick.rightBumper().onFalse(Commands.runOnce(()->drivetrain.setTarget(true)));


        //CLIMBER CONTROLS

        // driverJoystick.y().whileTrue(climber.raise());
        // driverJoystick.a().whileTrue(climber.lower());

        //readyToShoot.onFalse(shooter.stopMagazine().onlyIf(()->!shooter.isReversing()));

        //driverJoystick.a().onTrue((shooter.runMagazine()).andThen(shooter.runFlywheel()));
        //driverJoystick.a().onFalse(hopper.stop().andThen(shooter.stopMagazine()).andThen(shooter.idleFlywheel()));

        //left is back right is start
        //DRIVETRAIN RESETS
        driverJoystick.back().onTrue(new InstantCommand(()->drivetrain.seedFieldCentric()));
        driverJoystick.start().onTrue(new InstantCommand(()->drivetrain.visionOdoReset()));

        //KEYPAD TRIMS
        keypad.button(1).onTrue(Commands.runOnce(()->shooter.resetDistanceTrim()));
        keypad.button(2).onTrue(Commands.runOnce(()->shooter.trimLeft()));
        keypad.button(3).onTrue(Commands.runOnce(()->shooter.trimFurther()));
        keypad.button(4).onTrue(Commands.runOnce(()->shooter.trimCloser()));
        keypad.button(5).onTrue(Commands.runOnce(()->shooter.resetAngularTrim()));
        keypad.button(6).onTrue(Commands.runOnce(()->shooter.trimRight()));

        keypad.button(7).onTrue(hopper.forward())
        .onFalse(hopper.stop().onlyIf(()->!driverJoystick.rightBumper().getAsBoolean()));
        keypad.button(8).onTrue(hopper.stop());
        keypad.button(9).or(driverJoystick.rightTrigger(Constants.OperatorConstants.kTriggerThreshhold)).onTrue(hopper.backward().andThen(shooter.reverseMagazine()))
        .onFalse(hopper.forward().onlyIf(()->driverJoystick.rightBumper().getAsBoolean()))
        .onFalse(hopper.stop().onlyIf(()->!driverJoystick.rightBumper().getAsBoolean()))
        .onFalse(shooter.stopMagazine().onlyIf(()->!shooter.readyToShoot() || !driverJoystick.rightBumper().getAsBoolean()))
        .onFalse(shooter.runMagazine().onlyIf(()->shooter.readyToShoot() && driverJoystick.rightBumper().getAsBoolean()));


        keypad.button(10).or(driverJoystick.a()).whileTrue(drivetrain.applyRequest(()->brake));

        keypad.button(11).onTrue(intake.jiggle())
        .onFalse(intake.deploy());

        // magazineJam.whileTrue(hopper.backward()
        // .andThen(shooter.reverseMagazine())
        // .andThen(Commands.waitSeconds(.5))
        // .andThen((shooter.runMagazine().andThen(hopper.forward()).onlyIf(()->shooter.readyToShoot()))).repeatedly())
        // .onFalse((shooter.runMagazine().andThen(hopper.forward()).onlyIf(()->shooter.readyToShoot())))
        // .onFalse((shooter.stopMagazine().andThen(hopper.stop()).onlyIf(()->!shooter.readyToShoot())));
        
    }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
  public double getIntakeStartPoint(){
    return intakeChooser.getSelected();
  }
}
