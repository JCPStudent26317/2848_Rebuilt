// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Translation2d;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import lombok.Getter;
import lombok.Setter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  /**Xbox Controller IDs here.*/
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class IntakeConstants {
    public static final int kIntakeMotorID = 43;
    public static final double kIntakeMotorSpeed = 1.0;
    public static final int kPivotMotorID = 0;
    public static final double kPivotMotorSpeed = 1.0;    
  }

  public static class HopperConstants {
    // These names should be changed
    public static final int kBeltMotorID = 44;
    public static final double kBeltMotorSpeed = 0.66;    
    public static final int kRollerMotorID = 41;
    public static final double kRollerMotorSpeed = 0.66;
    public static final int kTransitionMotorID = 42;
    public static final double kTransitionMotorSpeed = 0.66;    
  }

  /**Constant values for the Shooter subsystem.*/
  public static class ShooterConstants {
    public static final int kFlywheelLeftMotorID = 50;
    public static final int kFlywheelRightMotorID = 51;
    public static final int kTurretMotorID = 48;
    public static final int kTurretCANcoderID = 47;
    public static final int kMagazineMotorID = 40;
    public static final double kMagazineMotorSpeed = .5;

    public static final double kTurretOffset = -.033;

    public static final double kFlywheelRPMMult = 1.5;

    public static final double kFlywheelRadius = .05; //meters

    public static final MagnetSensorConfigs kTurretCANcoderMagnetSensorConfigs = new MagnetSensorConfigs()
        .withMagnetOffset(-.033)
        .withAbsoluteSensorDiscontinuityPoint(0)
        .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);

    public static final double kFlywheelkS = 0.0005;
    public static final double kFlywheelkV = 0.002;
    public static final double kFlywheelkP = 0;//.000175;  
    public static final double kFlywheelkI = 0.0;
    public static final double kFlywheelkD = 0.000;
    public static final double kFlywheelkA = .0005;
    // the order of the pid values is different between these two !!
    public static final double kTurretkS = 0.2;
    public static final double kTurretkV = 5;
    public static final double kTurretkA = .2;
    public static final double kTurretkP = 30;
    public static final double kTurretkI = 0.3;
    public static final double kTurretkD = 0.00;    
    
    public static final double kFlywheelPeakVoltage = 16;
    public static final double kTurretPeakVoltage = 6;

    public static final double kTurretMMCruiseVelocity = 7;
    public static final double kTurretMMAcceleration = 14;
    public static final double kTurretMMJerk = 140;

    public static final double kTurretSwitchForwardLimit = 0.05;
    public static final double kTurretSwitchReverseLimit = -0.33;

  }

  /**Constant values for the Vision subsystem.*/
  public static class VisionConstants {

    /** Class for applying crop settings to LimeLights.*/

    /** Class for applying crop settings to LimeLights.*/
    public static class CropWindowSettings{
      @Getter @Setter private double cropXMin;
      @Getter @Setter private double cropXMax;
      @Getter @Setter private double cropYMin;
      @Getter @Setter private double cropYMax;

      public CropWindowSettings(double cropXMin, double cropXMax, double cropYMin, double cropYMax){
        if (cropXMin >= cropXMax || cropYMin >= cropYMax) {
          throw new IllegalArgumentException("Invalid crop window: min must be less than max");
        }
        this.cropXMin = cropXMin;
        this.cropXMax = cropXMax;
        this.cropYMin = cropYMin;
        this.cropYMax = cropYMax;
      }

      public CropWindowSettings(){
        this.cropXMin = -1;
        this.cropXMax = 1;
        this.cropYMin = -1;
        this.cropYMax = 1;
      }
    }

    // List of camera names published to the network tables (set in the limelight browser config tool)
    public static final String[] kCameraList = {"limelight-front"}; // limelight-left
    public static final boolean kAddToPoseEstimator = true;

    // Camera settings
    public static final int[] kRedAprilTagList = new int[]{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
    public static final int[] kBlueAprilTagList = new int[]{17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32};
    public static final int[] kAllAprilTagList = new int[]{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32};
    public static final float kDownscaleFactor = 4.0f;
    // crop settings
    public static final Map<String, CropWindowSettings> cameraCropWindowMap;

    static {
        cameraCropWindowMap = new HashMap<>();
        cameraCropWindowMap.put("limelight-right", new CropWindowSettings(-1, 1, -0.4, 0.9));
        cameraCropWindowMap.put("limelight-front", new CropWindowSettings());
    }

    // Position Constants
    public static final Translation2d kRobotToTurretTranslation = new Translation2d(-0.25, .508); // The position of the center of the turret fron the center of the robot with +x towards the front of the robot and +y towards the left of the robot
    public static final double kTurretToCameraMagnitude = .19; // The distance from the center of the turret to the Limelight

    // Filters
    public static final boolean kApplyFilters = true;
    public static final double kMinTagArea = 0.25;
    public static final double kMaxRotationalRate = 360; // deg/s
    public static final double kMaxTranslationalErrorMagnitude = 1; //m
    public static final double kMaxRotationalErrorMagnitude = 0.25; //rad
    public static final double kMaxAmbiguity = 0.6;
    // Can also be grabbed from WPI AprilTag class
    public static final double kFieldWidth = 8.05;
    public static final double kFieldLength = 17.55;

    // Standard Deviation 
    public static final double kInvalidStandardDeviation = 9999999;
    public static final double kMinimumTranslationalStandardDeviation = 0.5;
    public static final double kMinimumRotationalStandardDeviation = Math.toRadians(2.5); // rad
    public static final double kAddSkewDataDistanceThreshold = 2; //m
    }

  public static class ClimberConstants {
    public static final int kClimberMotorID = 0;
    public static final double kClimberMotorSpeed = 1;    
  }


}
