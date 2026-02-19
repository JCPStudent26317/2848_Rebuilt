package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import lombok.Getter;

import static frc.robot.generated.TunerConstants.*;

import java.util.ArrayList;



public class Avoider {
    @Getter private static ArrayList<Rectangle2d> rects = new ArrayList<>(){{
    }};

    public static final double kEpsilon = 1e-4; 
    public static final double kMaxDecel = 3.0; // Example: 3 meters per second squared
/**
 * Calculates independent scaling factors for X and Y field-relative speeds.
 * This allows the robot to "slide" along boundaries by only slowing the 
 * component of velocity that would cause a collision.
 */
public static Translation2d getInputScales(Pose2d pose, ChassisSpeeds speeds) {
    double vx = speeds.vxMetersPerSecond;
    double vy = speeds.vyMetersPerSecond;

    double scaleX = 1.0;
    double scaleY = 1.0;

    double curX = pose.getX();
    double curY = pose.getY();

    for (Rectangle2d zone : rects) {
        // Calculate the boundaries of the rectangle
        double minX = zone.getCenter().getX() - (zone.getXWidth() / 2.0);
        double maxX = zone.getCenter().getX() + (zone.getXWidth() / 2.0);
        double minY = zone.getCenter().getY() - (zone.getYWidth() / 2.0);
        double maxY = zone.getCenter().getY() + (zone.getYWidth() / 2.0);

        // Physics: distance needed to stop = v^2 / (2 * a)
        double brakingDistX = (vx * vx) / (2.0 * kMaxDecel);
        double brakingDistY = (vy * vy) / (2.0 * kMaxDecel);

        // --- X-AXIS CHECK (Sliding along North/South walls) ---
        // Only scale X if our current Y position is "blocked" by the rectangle's Y-span
        if (curY >= minY && curY <= maxY) {
            double distToEdgeX = -1.0;

            if (vx > kEpsilon && curX < minX) {
                distToEdgeX = minX - curX; // Approaching from the left
            } else if (vx < -kEpsilon && curX > maxX) {
                distToEdgeX = curX - maxX; // Approaching from the right
            }

            if (distToEdgeX >= 0) {
                // If our braking distance is greater than the gap, scale down
                if (brakingDistX > distToEdgeX) {
                    double candidateX = Math.sqrt(distToEdgeX / brakingDistX);
                    scaleX = Math.min(scaleX, candidateX);
                }
            } else if (curX >= minX && curX <= maxX) {
                // We are already inside the X-bounds of the rectangle
                // Don't allow moving deeper, but allow moving away.
                scaleX = 0.0; 
            }
        }

        // --- Y-AXIS CHECK (Sliding along East/West walls) ---
        // Only scale Y if our current X position is "blocked" by the rectangle's X-span
        if (curX >= minX && curX <= maxX) {
            double distToEdgeY = -1.0;

            if (vy > kEpsilon && curY < minY) {
                distToEdgeY = minY - curY; // Approaching from the bottom
            } else if (vy < -kEpsilon && curY > maxY) {
                distToEdgeY = curY - maxY; // Approaching from the top
            }

            if (distToEdgeY >= 0) {
                if (brakingDistY > distToEdgeY) {
                    double candidateY = Math.sqrt(distToEdgeY / brakingDistY);
                    scaleY = Math.min(scaleY, candidateY);
                }
            } else if (curY >= minY && curY <= maxY) {
                // We are already inside the Y-bounds
                scaleY = 0.0;
            }
        }
    }

    return new Translation2d(
        MathUtil.clamp(scaleX, 0.0, 1.0),
        MathUtil.clamp(scaleY, 0.0, 1.0)
    );
}

public static Translation2d getBrakePoint(ChassisSpeeds speeds){
    double vx = speeds.vxMetersPerSecond;
    double vy = speeds.vyMetersPerSecond;
    return new Translation2d(
        (vx * vx) / (2.0 * kMaxDecel),
        (vy * vy) / (2.0 * kMaxDecel)
    );
}

}