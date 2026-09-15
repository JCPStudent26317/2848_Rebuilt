package frc.robot;

import static frc.robot.Constants.VisionConstants.kMinTagArea;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.LimelightHelpers.PoseEstimate;
import lombok.Getter;
import lombok.Setter;

public class PoseEstHolder {
    @Setter @Getter PoseEstimate est = new PoseEstimate();
    @Setter @Getter double tagArea = 0;
    @Setter @Getter double tagCount = 0;
    @Setter @Getter private Matrix<N3, N1> stdevs = VecBuilder.fill(0,0,0);
    @Setter @Getter private double targetSkewDegrees = 0;
    @Setter @Getter private double adjustedSkewAngle = 0;
    @Getter private final String cameraName;
    @Setter @Getter boolean valid = false;
    @Getter RollingAverage fpsRoller = new RollingAverage(1);
    @Getter RollingAverage ambiguityRoller = new RollingAverage(1);


    public PoseEstHolder(String name){
     this.cameraName = name;
    }

    public boolean hasTag(){
        return kMinTagArea < NetworkTableInstance.getDefault().getTable(cameraName).getEntry("botpose").getDoubleArray(new double[11])[10];
    }

    public void update(){
        double now = Timer.getFPGATimestamp();
        fpsRoller.update(new double[]{now, NetworkTableInstance.getDefault().getTable(cameraName).getEntry("hw").getDoubleArray(new double[4])[3]});
        ambiguityRoller.update(new double[]{now,this.est.rawFiducials[0].ambiguity});
    }
    

}
