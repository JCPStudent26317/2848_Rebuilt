package frc.robot;

import static frc.robot.Constants.VisionConstants.kMinTagArea;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
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


    public PoseEstHolder(String name){
     this.cameraName = name;
    }

    public boolean hasTag(){
        return est.tagCount >0;
    }
    

}
