// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
    public static final int kLRollersMotorID = 15;
    public static final int kRRollersMotorID = 16;
    public static final int kIntakePivotID = 17;
    public static final int kIntakePivotCANcoderID = 18;
    public static final double kRollersMotorSpeed = 1.0;
    public static final int kPivotMotorID = 0;
    public static final double kPivotMotorSpeed = 1.0;    

    public static final double kPivotkS = 0;
    public static final double kPivotkV = 0;
    public static final double kPivotkA = 0;
    public static final double kPivotkP = 36;
    public static final double kPivotkI = 0;
    public static final double kPivotkD = 0;

    public static final double kPivotMMCruiseVelocity = 7;
    public static final double kPivotMMAcceleration = 14;
    public static final double kPivotMMJerk = 140;
    
    public static final double kDeploySetpoint = 0.573486328125;
    public static final double kLowRetractSetpoint = 0.65;
    public static final double kHighRetractSetpoint = 0.75;
    public static final double kStowSetpoint = 0.96;


     public static final MagnetSensorConfigs kIntakeCANcoderMagnetSensorConfigs = new MagnetSensorConfigs()
        .withMagnetOffset(0.10)
        .withAbsoluteSensorDiscontinuityPoint(1)
        .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);
  }

  public static class HopperConstants {
    public static final int kSidewaysBeltMotorID = 21;
    public static final double kSidewaysBeltSpeed = 1.0;    
    public static final int kForwardBeltMotorID = 22;
    public static final double kForwardBeltSpeed = 1.0;    
  }

  /**Constant values for the Shooter subsystem.*/
  public static class ShooterConstants {
    public static final int kFlywheelLeftMotorID = 28;
    public static final int kFlywheelRightMotorID = 29;
    public static final int kTurretMotorID = 25;
    public static final int kTurretCANcoderID = 26;
    public static final int kMagazineMotorID = 27;
    public static final double kMagazineMotorSpeed = .5;

    public static final double kTurretOffset = -.033;

    public static final double kFlywheelRPMMult = 1.5; //multiplier on flywheel speed to account for slipping between flywheel and fuel

    public static final double kFlywheelRadius = .05; //meters

    public static final double kFlywheelIdleSpeed = 200;

    public static final MagnetSensorConfigs kTurretCANcoderMagnetSensorConfigs = new MagnetSensorConfigs()
        .withMagnetOffset(-0.009)
        .withAbsoluteSensorDiscontinuityPoint(.5)
        .withSensorDirection(SensorDirectionValue.Clockwise_Positive);

    public static final double kFlywheelkS = 0.0005;
    public static final double kFlywheelkV = 0.002;
    public static final double kFlywheelkP = .0001;  
    public static final double kFlywheelkI = 0.0;
    public static final double kFlywheelkD = 0.000;
    public static final double kFlywheelkA = .0005;
    // the order of the pid values is different between these two !!
    public static final double kTurretkS = 0.05;
    public static final double kTurretkV = 5;
    public static final double kTurretkA = .1;
    public static final double kTurretkP = 30;//30;
    public static final double kTurretkI = 1;//0.01;
    public static final double kTurretkD = 0.00;    

    public static final double kTurretCorrectionkV = .5;
    public static final double kTurretCorrectionkS =.1;
    
    public static final double kFlywheelPeakVoltage = 16;
    public static final double kTurretPeakVoltage = 6;

    public static final double kTurretMMCruiseVelocity = 14;
    public static final double kTurretMMAcceleration = 28;
    public static final double kTurretMMJerk = 280;

    public static final double kTurretSwitchForwardLimit = 0.25;
    public static final double kTurretSwitchReverseLimit = -0.44;

    public static final double kFlywheelRPMTolerance = 50;
    public static final double kTurretPositionTolerance = .01;

  }

  public static class drivePoints {
    public static final Pose2d redOutpostClimb = new Pose2d(15.4,5,new Rotation2d(Math.toRadians(90)));
    public static final Pose2d redOutpostClimbLineup = new Pose2d(15.4,5.852,new Rotation2d(Math.toRadians(90)));

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
    public static final String[] kCameraList = {"limelight-back","limelight-left","limelight-right","limelight-turret"}; // limelight-left
    public static final boolean kAddToPoseEstimator = true;

    // Camera settings
    public static final int[] kRedAprilTagList = new int[]{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
    public static final int[] kBlueAprilTagList = new int[]{17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32};
    public static final int[] kAllAprilTagList = new int[]{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32};
    public static final int[] kTowerAprilTagList = new int[]{15,16,31,32};
    public static final float kDownscaleFactor = 4.0f;
    // crop settings
    public static final Map<String, CropWindowSettings> cameraCropWindowMap;

    static {
        cameraCropWindowMap = new HashMap<>();
        cameraCropWindowMap.put("limelight-left", new CropWindowSettings());
        cameraCropWindowMap.put("limelight-back", new CropWindowSettings());
        cameraCropWindowMap.put("limelight-right", new CropWindowSettings());
        cameraCropWindowMap.put("limelight-turret", new CropWindowSettings());
    }

    // Position Constants

    //TODO: update the translations
    public static final Translation2d kRobotToTurretTranslation = new Translation2d(-0.178, .178); // The position of the center of the turret fron the center of the robot with +x towards the front of the robot and +y towards the left of the robot
    public static final double kTurretToCameraMagnitude = .1; // The distance from the center of the turret to the Limelight

    // Filters
    public static final boolean kApplyFilters = true;
    public static final double kMinTagArea = 0.25;
    public static final double kMaxRotationalRate = 360; // deg/s
    public static final double kMaxTranslationalErrorMagnitude = 1; //m
    public static final double kMaxRotationalErrorMagnitude = 0.25; //rad
    public static final double kMaxAmbiguity = 0.6;
    // Can also be grabbed from WPI AprilTag class
    public static final double kFieldWidth = 8.042656;
    public static final double kFieldLength = 16.513048;

    // Standard Deviation 
    public static final double kInvalidStandardDeviation = 9999999;
    public static final double kMinimumTranslationalStandardDeviation = 0.5;
    public static final double kMinimumRotationalStandardDeviation = Math.toRadians(5); // rad
    public static final double kAddSkewDataDistanceThreshold = 2; //m
    }

  public static class ClimberConstants {
    public static final int kClimberMotorID = 33;
    public static final double kClimberMotorSpeed = 1;    
  }


}
