// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {

    for(String i : Constants.VisionConstants.kCameraList){
      NetworkTable table = NetworkTableInstance.getDefault().getTable(i);
      table.getEntry("throttle_set").setNumber(150);

      SmartDashboard.putNumber(""+ i +" Temp (C)",table.getEntry("hw").getDoubleArray(new double[3])[0]);
    }





    if(!m_robotContainer.getShooter().sanityCheck()){
      DriverStation.reportError("Something in the shooter is disconnected / Check Phoenix Tuner",false);
    }
    if(!m_robotContainer.getIntake().sanityCheck()){
      DriverStation.reportError("Something in the intake is disconnected / Check Phoenix Tuner",false);
    }
    if(!m_robotContainer.getLights().sanityCheck()){
      DriverStation.reportError("Something in the lights is disconnected / Check Phoenix Tuner",false);
    }
    if(!m_robotContainer.getHopper().sanityCheck()){
      DriverStation.reportError("Something in the hopper is disconnected / Check Phoenix Tuner",false);
    }
    String newAutoName = m_robotContainer.getAutonomousCommand().getName();
    String autoName = "";
        if (autoName != newAutoName) {
        autoName = newAutoName;
        if (AutoBuilder.getAllAutoNames().contains(autoName)) {
            //System.out.println("Displaying " + autoName);
            try{
            List<PathPlannerPath> pathPlannerPaths = PathPlannerAuto.getPathGroupFromAutoFile(autoName);
            List<Pose2d> poses = new ArrayList<>();
            for (PathPlannerPath path : pathPlannerPaths) {
                poses.addAll(path.getAllPathPoints().stream().map(point -> new Pose2d(point.position.getX(), point.position.getY(), new Rotation2d())).collect(Collectors.toList()));
            }
            m_robotContainer.getDrivetrain().getM_field().getObject("path").setPoses(poses);
          } catch (Exception e){
            System.out.println(e);
          }
        }
  }
}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
    for(String i : Constants.VisionConstants.kCameraList){
      NetworkTable table = NetworkTableInstance.getDefault().getTable(i);
      table.getEntry("throttle_set").setNumber(0);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {

    for(String i : Constants.VisionConstants.kCameraList){
      NetworkTable table = NetworkTableInstance.getDefault().getTable(i);
      table.getEntry("throttle_set").setNumber(0);
    }
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.getIntake().gotoStartPoint();

    CommandScheduler.getInstance().schedule(m_robotContainer.getStopShoot());
    //m_robotContainer.getDrivetrain().getM_field().getObject("path").close();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
