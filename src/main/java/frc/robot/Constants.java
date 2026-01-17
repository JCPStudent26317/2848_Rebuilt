// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;
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

  public static class ShooterConstants {
    public static final int kFlywheelMotorID = 10;
    public static final int kHoodMotorID = 11;
    public static final int kTurretMotorID = 12;

  }

  //**Just copied from 2025 project.*/
  public static class VisionConstants {
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
    public static final String[] kCameraList = {"limelight-right"}; // limelight-left
    public static final boolean kAddToPoseEstimator = true;

    // Camera settings
    public static final int[] kRedAprilTagList = new int[]{6, 7, 8, 9, 10, 11};
    public static final int[] kBlueAprilTagList = new int[]{17, 18, 19, 20, 21, 22};
    public static final int[] kAllAprilTagList = new int[]{6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};
    public static final float kDownscaleFactor = 4.0f;
    // crop settings
    public static final Map<String, CropWindowSettings> cameraCropWindowMap;

    static {
        cameraCropWindowMap = new HashMap<>();
        cameraCropWindowMap.put("limelight-right", new CropWindowSettings(-1, 1, -0.4, 0.9));
        cameraCropWindowMap.put("limelight-left", new CropWindowSettings());
    }


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

}
