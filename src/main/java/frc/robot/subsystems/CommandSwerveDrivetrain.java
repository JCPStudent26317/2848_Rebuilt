package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.VisionConstants.kFieldLength;
import static frc.robot.Constants.VisionConstants.kFieldWidth;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import lombok.Setter;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private FieldCentric autoAlignRequest = new SwerveRequest.FieldCentric();

    private Field2d m_field = new Field2d();

    private final ProfiledPIDController autoAlignXController = new ProfiledPIDController(TunerConstants.autoAlign_Translation_P, TunerConstants.autoAlign_Translation_I, TunerConstants.autoAlign_Translation_D, 
                                                                                    new TrapezoidProfile.Constraints(TunerConstants.autoAlign_Translation_maxVx, TunerConstants.autoAlign_Translation_MaxA));
    private final ProfiledPIDController autoAlignYController = new ProfiledPIDController(TunerConstants.autoAlign_Translation_P, TunerConstants.autoAlign_Translation_I, TunerConstants.autoAlign_Translation_D, 
                                                                                    new TrapezoidProfile.Constraints(TunerConstants.autoAlign_Translation_maxVy, TunerConstants.autoAlign_Translation_MaxA));
    private final ProfiledPIDController autoAlignRotationController = new ProfiledPIDController(TunerConstants.autoAlign_Rotation_P, TunerConstants.autoAlign_Rotation_I, TunerConstants.autoAlign_Rotation_D, 
                                                                                    new TrapezoidProfile.Constraints(TunerConstants.autoAlign_Rotation_maxV, TunerConstants.autoAlign_Rotation_MaxA));
    private int redAutoAlign = 1;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    private Translation2d targetPos = new Translation2d();
    private Translation2d hubPos = new Translation2d();

    private boolean redAlliance = false;

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    /** Swerve request to apply during robot-centric path following */
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
     * @param modules               Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

            init();
            configureAutoBuilder(); 
    }

    public void visionOdoReset(){
        PoseEstimate poseEstimate = RobotContainer.getVision().getVisionPoseEstimate();
        if (poseEstimate != null){
            this.resetPose(poseEstimate.pose);
        }
    }

    public void init(){

        autoAlignRotationController.enableContinuousInput(-Math.PI, Math.PI);

        autoAlignXController.setTolerance(TunerConstants.autoAlign_Translation_Tolerance);
        autoAlignYController.setTolerance(TunerConstants.autoAlign_Translation_Tolerance);
        autoAlignRotationController.setTolerance(TunerConstants.autoAlign_Rotation_Tolerance);

        this.register();



        if (DriverStation.getAlliance().get() == Alliance.Red){
            targetPos = Constants.ShooterConstants.redHubPose;
            redAutoAlign =-1;
            redAlliance = true;
        } else{
            targetPos = Constants.ShooterConstants.blueHubPose;
            redAutoAlign = 1;
            redAlliance = false;
        }
        hubPos = targetPos;
        SmartDashboard.putString("TargetPos",targetPos.toString());

    }



    public void setTarget(boolean aimAtHub){
        Pose2d pos = this.getState().Pose;
        if(aimAtHub){
            targetPos = hubPos;
        } else if(redAlliance && pos.getY()>kFieldWidth/2){
            targetPos = Constants.ShooterConstants.redOutpostCornerPose;
        } else if(redAlliance && pos.getY()<=kFieldWidth/2){
            targetPos = Constants.ShooterConstants.redDepotCornerPose;
        } else{
            if(pos.getY()>kFieldWidth/2){
                targetPos = Constants.ShooterConstants.blueDepotCornerPose;
            } else{
                targetPos = Constants.ShooterConstants.blueOutpostCornerPose;
            }
        }
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        init();
        configureAutoBuilder();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        init();
        configureAutoBuilder();
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }
/**
 * gets the distance from the center of the robot and target
 * @return distance in meters
 */
    public double getTargetDist(){
        return targetPos.minus(this.getState().Pose.getTranslation()).getNorm();
    }
/**
 * gets the theta between robot front and target translation
 * @return angle in radians
 */
        public double getTargetTheta() {
        Pose2d robotPose = getState().Pose;

        Translation2d toTarget =
            targetPos.minus(robotPose.getTranslation());

        double targetAngle = toTarget.getAngle().getRadians();
        double robotAngle  = robotPose.getRotation().getRadians();

        return MathUtil.angleModulus(targetAngle - robotAngle);
    }

    public ChassisSpeeds getFieldOrientedSpeeds(){
        return ChassisSpeeds.fromRobotRelativeSpeeds(this.getState().Speeds,this.getState().Pose.getRotation());
    }

/**
 * gets the polar velocities around a point
 * @return translation2d of radial, tangential
 */
    public Translation2d getPolarVelocity(){
        Translation2d r = targetPos.minus(this.getState().Pose.getTranslation());
        double dist = getTargetDist();

        // Unit radial vector
        double rx = r.getX() / dist;
        double ry = r.getY() / dist;

        // Unit tangential vector (90° CCW rotation)
        double tx = ry;
        double ty = -rx;

        // Robot velocity components
        double vx = getFieldOrientedSpeeds().vxMetersPerSecond;
        double vy = getFieldOrientedSpeeds().vyMetersPerSecond;

        // Dot products → projections
        double vRadial = vx * rx + vy * ry;
        double vTangential = vx * tx + vy * ty;

        // Return as Translation2d: x = radial, y = tangential
        return new Translation2d(vRadial, vTangential);

    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }
        m_field.setRobotPose(this.getState().Pose);
      
        SmartDashboard.putData("Field",m_field);
        SmartDashboard.putNumber("hub theta",getTargetTheta());
        SmartDashboard.putNumber("tangential velocity",getPolarVelocity().getY());
        SmartDashboard.putNumber("radial velocity",getPolarVelocity().getX());
        SmartDashboard.putNumber("field velocity",getVelocityMag());
        // Print whether the pathplanner auto should be flipped
        SmartDashboard.putBoolean("Flipped PathPlanner", DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red);
    }



/**
 * 
 * @return magnitude of the translational velocity in m/s
 */
    public double getVelocityMag(){
       return Math.hypot(this.getState().Speeds.vxMetersPerSecond,this.getState().Speeds.vyMetersPerSecond);
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement
     *     in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }

    public Command autoAlignTo(Pose2d endPose){
        return Commands.startRun(()->{
            autoAlignXController.reset(this.getState().Pose.getX(),getFieldOrientedSpeeds().vxMetersPerSecond);
            autoAlignYController.reset(this.getState().Pose.getY(),getFieldOrientedSpeeds().vyMetersPerSecond);
            autoAlignRotationController.reset(this.getState().Pose.getRotation().getRadians(),getFieldOrientedSpeeds().omegaRadiansPerSecond);

            //Pose2d poseError = this.getState().Pose.relativeTo(endPose);

            autoAlignXController.setGoal(endPose.getX());
            autoAlignYController.setGoal(endPose.getY());
            autoAlignRotationController.setGoal(endPose.getRotation().getRadians());
        },
        ()->{
            Pose2d pose = this.getState().Pose;

            ChassisSpeeds out = new ChassisSpeeds(
                autoAlignXController.calculate(pose.getX()) *redAutoAlign,
                autoAlignYController.calculate(pose.getY()) *redAutoAlign,
                autoAlignRotationController.calculate(pose.getRotation().getRadians())
            );

            autoAlignRequest
            .withVelocityX(out.vxMetersPerSecond)
            .withVelocityY(out.vyMetersPerSecond)
            .withRotationalRate(out.omegaRadiansPerSecond);
            this.setControl(autoAlignRequest);
        }).until(()->autoAlignAtGoal()).withName("Auto Align");
    }

    private boolean autoAlignAtGoal(){
        return autoAlignXController.atGoal() && autoAlignYController.atGoal() && autoAlignRotationController.atGoal();
    }

/**
 * finds the correct set of auto align commands to climb based on field position
 * @return the sequential command group of auto align commands which drive the robot to the climb spot
 */
    public Command autoAlignClimb(){
        if(redAlliance && this.getState().Pose.getY()>Constants.VisionConstants.kFieldWidth/2){
            return autoAlignTo(Constants.drivePoints.redOutpostClimbLineup).andThen(autoAlignTo(Constants.drivePoints.redOutpostClimb));
        } else if (redAlliance && this.getState().Pose.getY()<Constants.VisionConstants.kFieldWidth/2){
            return autoAlignTo(Constants.drivePoints.redDepotClimbLineup).andThen(autoAlignTo(Constants.drivePoints.redDepotClimb));
        }else if (!redAlliance && this.getState().Pose.getY()>Constants.VisionConstants.kFieldWidth/2){
            return autoAlignTo(Constants.drivePoints.blueDepotClimbLineup).andThen(autoAlignTo(Constants.drivePoints.blueDepotClimb));
        } else{
            return autoAlignTo(Constants.drivePoints.blueOutpostClimbLineup).andThen(autoAlignTo(Constants.drivePoints.blueOutpostClimb));
        }
    }



    private void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> getState().Pose,   // Supplier of current robot pose
                this::resetPose,         // Consumer for seeding pose against auto
                () -> getState().Speeds, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds, feedforwards) -> setControl(
                    m_pathApplyRobotSpeeds.withSpeeds(ChassisSpeeds.discretize(speeds, 0.020))
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    // PID constants for translation
                    new PIDConstants(10, 0, 0),
                    // PID constants for rotation
                    new PIDConstants(7, 0, 0)
                ),
                config,
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }    

}
