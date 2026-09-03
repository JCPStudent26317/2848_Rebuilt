package frc.robot.subsystems;

import static frc.robot.Constants.VisionConstants.cameraCropWindowMap;
import static frc.robot.Constants.VisionConstants.kAddSkewDataDistanceThreshold;
import static frc.robot.Constants.VisionConstants.kAllAprilTagList;
import static frc.robot.Constants.VisionConstants.kCameraList;
import static frc.robot.Constants.VisionConstants.kDownscaleFactor;
import static frc.robot.Constants.VisionConstants.kFieldLength;
import static frc.robot.Constants.VisionConstants.kFieldWidth;
import static frc.robot.Constants.VisionConstants.kInvalidStandardDeviation;
import static frc.robot.Constants.VisionConstants.kMaxAmbiguity;
import static frc.robot.Constants.VisionConstants.kMaxRotationalRate;
import static frc.robot.Constants.VisionConstants.kMinimumRotationalStandardDeviation;
import static frc.robot.Constants.VisionConstants.kMinimumTranslationalStandardDeviation;
import static frc.robot.Constants.VisionConstants.kRobotToTurretTranslation;
import static frc.robot.Constants.VisionConstants.kTurretToCameraMagnitude;

import java.util.Arrays;
import java.util.HashMap;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants.CropWindowSettings;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.PoseEstHolder;

public class New_Vision extends SubsystemBase{


    //#region init
    private String[] cameraList;

    private boolean useOldStdDev = true;

    private HashMap<String,PoseEstHolder> cameraMap = new HashMap<>();

    public New_Vision(){
        this.cameraList = kCameraList;
        this.register();
        configureCameras();
        for (String camera : cameraList){
            cameraMap.put(camera,new PoseEstHolder(camera));
        }
    }
    
    public void configureCameras(){
        for (String camera : cameraList){
            configureCamera(camera);
        }
    }
    private void configureCamera(String camera){
        LimelightHelpers.SetFiducialIDFiltersOverride(camera, kAllAprilTagList); // Only track these tag IDs
        //TODO: try new downscales for more range?
        LimelightHelpers.SetFiducialDownscalingOverride(camera, kDownscaleFactor); // Increases the framerate

        // Force LEDs off
        LimelightHelpers.setLEDMode_ForceOff(camera);


        // Apply window crop settings to increase framerate
        CropWindowSettings cropWindow = cameraCropWindowMap.get(camera);
        LimelightHelpers.setCropWindow(camera, cropWindow.getCropXMin(), cropWindow.getCropXMax(), cropWindow.getCropYMin(), cropWindow.getCropYMax());
    }

    //#endregion

    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        
        builder.addDoubleProperty(cameraList[0] + "Last update",
        ()->cameraMap.get(cameraList[0]).getEst().timestampSeconds,
        null);
        
        builder.addDoubleProperty(cameraList[1] + "Last update",
        ()->cameraMap.get(cameraList[1]).getEst().timestampSeconds,
        null);

        builder.addDoubleProperty(cameraList[2] + "Last update",
        ()->cameraMap.get(cameraList[2]).getEst().timestampSeconds,
        null);

        builder.addDoubleProperty(cameraList[3] + "Last update",
        ()->cameraMap.get(cameraList[3]).getEst().timestampSeconds,
        null);


        builder.addBooleanProperty(cameraList[0] + "Has Tag",
        ()->cameraMap.get(cameraList[0]).hasTag(),
        null);
        
        builder.addBooleanProperty(cameraList[1] + "Has Tag",
        ()->cameraMap.get(cameraList[1]).hasTag(),
        null);

        builder.addBooleanProperty(cameraList[2] + "Has Tag",
        ()->cameraMap.get(cameraList[2]).hasTag(),
        null);

        builder.addBooleanProperty(cameraList[3] + "Has Tag",
        ()->cameraMap.get(cameraList[3]).hasTag(),
        null);


        builder.addBooleanProperty(cameraList[0] + "Is Valid",
        ()->cameraMap.get(cameraList[0].IsValid()),
        null);

        builder.addBooleanProperty(cameraList[1] + "Is Valid",
        ()->cameraMap.get(cameraList[1].IsValid()),
        null);

        builder.addBooleanProperty(cameraList[2] + "Is Valid",
        ()->cameraMap.get(cameraList[2].IsValid()),
        null);

        builder.addBooleanProperty(cameraList[3] + "Is Valid",
        ()->cameraMap.get(cameraList[3].IsValid()),
        null);

    }


    @Override
    public void periodic(){
        //update and applies each new reading to each cameras specfic holder
        for (String camera : cameraList) {
            PoseEstHolder currentCam = cameraMap.get(camera);
            PoseEstimate visionPoseEstimate = currentCam.getEst();
            boolean hasTag = currentCam.hasTag();
            boolean rejectUpdate = false;
            if (hasTag){
                double[] botpose = NetworkTableInstance.getDefault()
                    .getTable(camera)
                    .getEntry("botpose")
                    .getDoubleArray(new double[11]);

                double tagArea = botpose[10];
                currentCam.setTagArea(tagArea);
                int tagCount = (int) botpose[7];
                cameraMap.get(camera).setTagCount(tagCount);
                    try{
                        if(camera.equals("limelight-turret")){
                    currentCam.setEst(getTurretToRobotPose(LimelightHelpers.getBotPoseEstimate_wpiBlue(camera)));
                    currentCam.setTargetSkewDegrees(LimelightHelpers.getTargetPose_RobotSpace(camera)[4]);
                    currentCam.setAdjustedSkewAngle(1 / Math.cos(Math.toRadians(currentCam.getTargetSkewDegrees())) - 1);
                        } else{
                            currentCam.setEst(LimelightHelpers.getBotPoseEstimate_wpiBlue(camera));
                            currentCam.setTargetSkewDegrees(LimelightHelpers.getTargetPose_RobotSpace(camera)[4]);
                            currentCam.setAdjustedSkewAngle(1 / Math.cos(Math.toRadians(currentCam.getTargetSkewDegrees())) - 1);
                        }
                    } catch(Exception e) {
                        System.out.println(e);
                    }
                //general sanity checks to filter bad readings
            } else{
                rejectUpdate = true;
            }
            if (visionPoseEstimate == null || visionPoseEstimate.rawFiducials.length ==0) {
                rejectUpdate = true;
            }
            // Reject update if our angular velocity is greater than a threshold degrees per second
            else if (Math.abs(RobotContainer.getDrivetrain().getPigeon2().getAngularVelocityZWorld().getValueAsDouble()) > kMaxRotationalRate){
                rejectUpdate = true;
            }
            // Reject update if there are no visible tags (Redundant?)
            else if (visionPoseEstimate.tagCount <1) {
                rejectUpdate = true;
            }
            // Reject update if the translational error magnitude is larger than the threshold
            // else if (Math.hypot(poseError.getX(), poseError.getY()) > kMaxTranslationalErrorMagnitude){
            //     rejectUpdate = true;
            // }
            // Reject update if the ambiguity is larger than the threshold
            else if (visionPoseEstimate.rawFiducials[0].ambiguity > kMaxAmbiguity){
                rejectUpdate = true;
            }
            // Reject update if the rotational error magnitude is larger than the threshold
            // else if (Math.abs(poseError.getRotation().getRadians()) > kMaxRotationalErrorMagnitude){
            //     rejectUpdate = true;
            // }
            // Reject update if the pose estimate is not inside of the field
            else if (visionPoseEstimate.pose.getX() < 0.0 || visionPoseEstimate.pose.getY() < 0.0 || 
                        visionPoseEstimate.pose.getX() > kFieldLength || visionPoseEstimate.pose.getY() > kFieldWidth){
                rejectUpdate = true;
            }
            currentCam.setValid(!rejectUpdate);
        }
        //detect outliers
        // int[][] outliers = detectOutlier(new PoseEstHolder[]{
        //     cameraMap.get(cameraList[0]),
        //     cameraMap.get(cameraList[1]),
        //     cameraMap.get(cameraList[2]),
        //     cameraMap.get(cameraList[3])
        // });

        // for (int i =0; i<cameraList.length; i++){
        //     if(outliers[0][i] == 0 || outliers[1][i] == 0){
        //         cameraMap.get(cameraList[i]).setValid(false);
        //     }
        // }
        //apply good readings
        for (String camera : cameraList){
            if(cameraMap.get(camera).isValid()){
                PoseEstimate est = cameraMap.get(camera).getEst();
                setStandardDeviation(camera);
                RobotContainer.getDrivetrain().addVisionMeasurement(
                    est.pose,
                    est.timestampSeconds,
                    cameraMap.get(camera).getStdevs()
                );
            }
        }
    }


    private int[][] detectOutlier(PoseEstHolder[] vals){

        int[] xOutliers = new int[4];
        int[] yOutliers = new int[4];
        Arrays.fill(xOutliers, -1);
        Arrays.fill(yOutliers,-1);
        double[] xs = {vals[0].getEst().pose.getX(),vals[1].getEst().pose.getX(),vals[2].getEst().pose.getX(),vals[3].getEst().pose.getX()};
        double[] ys = {vals[0].getEst().pose.getY(),vals[1].getEst().pose.getY(),vals[2].getEst().pose.getY(),vals[3].getEst().pose.getY()};
        
        
        double xMedian = median(xs);
        double yMedian = median(ys);

        for (int i =0; i<4;i++){
            if(xMedian +.3 < xs[i] || xMedian -.3 > xs[i]){
                xOutliers[i] = 0;
            }
            if(yMedian +.3 < xs[i] || yMedian -.3 > xs[i]){
                yOutliers[i] =0;
            }
        }



        return new int[][]{xOutliers, yOutliers};
    }

    private double median(double[] vals){
        Arrays.sort(vals);
        return (vals[1]+vals[2])/2;
    }


    public void setStandardDeviation(String camera){
        PoseEstimate visionPoseEstimate = cameraMap.get(camera).getEst();
        double translationStdDev=kMinimumTranslationalStandardDeviation;
        double rotationStdDev = kMinimumRotationalStandardDeviation;
        double adjustedSkewAngle = cameraMap.get(camera).getAdjustedSkewAngle();
        if (visionPoseEstimate.tagCount > 1){
            // If the camera sees more than 1 april tag assume the lowest standard deviation
            translationStdDev = kMinimumTranslationalStandardDeviation;
            rotationStdDev = kMinimumRotationalStandardDeviation;
        }
        else if (useOldStdDev){
            translationStdDev = (visionPoseEstimate.avgTagArea * (-18.3)) + 11.34;
            rotationStdDev = 0.3 * visionPoseEstimate.avgTagDist - 0.1;

            // Modify standard deviations using tag skew data if past the distance threshold
            if (visionPoseEstimate.avgTagDist > kAddSkewDataDistanceThreshold){
                rotationStdDev += 0.02 * (adjustedSkewAngle - 4.1) * (adjustedSkewAngle - 4.1);
            }
        }
        else{
            // Set standard deviations using avg tag distance data
            //limited to more than 0.5 because of the sqrt function
            if (visionPoseEstimate.avgTagDist > 0.5){
                translationStdDev = 0.25 * 1.2 * Math.sqrt(visionPoseEstimate.avgTagDist - 0.5);
                rotationStdDev = 0.3 * visionPoseEstimate.avgTagDist - 0.1;
            }
            
            // Modify standard deviations using tag skew data if past the distance threshold
            if (visionPoseEstimate.avgTagDist > kAddSkewDataDistanceThreshold){
                translationStdDev += 1.4 * (adjustedSkewAngle - 1.1) * (adjustedSkewAngle - 1.1);
                rotationStdDev += 0.02 * (adjustedSkewAngle - 4.1) * (adjustedSkewAngle - 4.1);
            }
        }

        // Check to make sure standard deviations are not below the minimum limits
        if (translationStdDev < kMinimumTranslationalStandardDeviation){
            translationStdDev = kMinimumTranslationalStandardDeviation;
        }
        if (rotationStdDev < kMinimumRotationalStandardDeviation){
            rotationStdDev = kMinimumRotationalStandardDeviation;
        }

        // becuase we use the gyro to figure out the pose from the turret ll we don't want to feed that back in
        if (camera.equals("limelight-turret")){
            cameraMap.get(camera).setStdevs(VecBuilder.fill(translationStdDev, translationStdDev, kInvalidStandardDeviation));
        }
        else{
            cameraMap.get(camera).setStdevs(VecBuilder.fill(translationStdDev, translationStdDev, rotationStdDev));
        }
        


        SmartDashboard.putNumber("Old Standard Deviation", (visionPoseEstimate.avgTagArea * (-18.3)) + 11.34);

        // Fill out Standard deviation matrix for drivebase
       
    }
    public PoseEstimate getTurretToRobotPose(PoseEstimate turretCameraPose){
        PoseEstimate robotPose = turretCameraPose;

        Translation2d robotToCameraTranslation = kRobotToTurretTranslation.plus(new Translation2d(kTurretToCameraMagnitude, new Rotation2d(RobotContainer.getShooter().getTurretAngle()))); // Robot oriented
        
        // Might need a plus pi for the red side
        robotPose.pose = new Pose2d(turretCameraPose.pose.getTranslation().minus(robotToCameraTranslation.rotateBy(RobotContainer.getDrivetrain().getState().Pose.getRotation())), RobotContainer.getDrivetrain().getState().Pose.getRotation());
        
        return robotPose;
    }

}
