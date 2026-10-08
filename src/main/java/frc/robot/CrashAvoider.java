package frc.robot;

import java.awt.geom.Line2D;
import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import lombok.Getter;

public class CrashAvoider {
    private double maxAcceleration = 3; // m/s^2
    private double robotLength = Units.inchesToMeters(15);//15.5
    private double robotWidth = Units.inchesToMeters(15);
    private double robotLengthIntakeDown = Units.inchesToMeters(25);

    private @Getter boolean clampX = false;
    private @Getter boolean clampY = false;

    private final Field2d debugField = new Field2d();

public CrashAvoider() {
    SmartDashboard.putData("CrashAvoider", debugField);
}

private void drawRays(Line2D[] rays, double angle, Pose2d robotPos) {
    List<Pose2d> starts = new ArrayList<>();
    List<Pose2d> ends = new ArrayList<>();
    for (Line2D r : rays) {
        starts.add(new Pose2d(r.getX1(), r.getY1(), new Rotation2d(angle)));
        ends.add(new Pose2d(r.getX2(), r.getY2(), new Rotation2d(angle)));
    }
    debugField.setRobotPose(robotPos);
    debugField.getObject("rayStarts").setPoses(starts);
    debugField.getObject("rayEnds").setPoses(ends);
}

    private static final double MARGIN = Units.inchesToMeters(3);
private static final double LATENCY = 0.08;      // s: loop + pose + motor response; tune
private static final double MIN_LOOKAHEAD = 0.5;  // m
private static final double EPS = 1e-6;

public SwerveRequest update(SwerveRequest.FieldCentric requested, Pose2d robotPos, ChassisSpeeds fieldSpeeds, boolean intakeDown) {
    // Request is in operator perspective; flip to blue-origin field frame on red
    boolean red = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    double sign = red ? -1 : 1;
    double reqVx = sign * requested.VelocityX;
    double reqVy = sign * requested.VelocityY;

    double reqSpeed = Math.hypot(reqVx, reqVy);
    if (reqSpeed < 1e-3) return requested;

    double measSpeed = Math.hypot(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);
    double v = Math.max(reqSpeed, measSpeed);
    double lookahead = Math.max(MIN_LOOKAHEAD, v * v / (2 * maxAcceleration) + v * LATENCY + MARGIN);
    double angle = Math.atan2(reqVy, reqVx);

    SmartDashboard.putBoolean("Intake down", intakeDown);

    Hit closest = null;
    for (Line2D ray : getVectors(lookahead, angle, robotPos, intakeDown)) {
        Hit h = closestHit(ray);
        if (h != null && (closest == null || h.distance() < closest.distance())) closest = h;
    }
    SmartDashboard.putBoolean("CA hit", closest != null);
    if (closest == null) return requested;

    // Largest v where (v * latency) + (v^2 / 2a) still fits in the remaining distance
    double a = maxAcceleration;
    double d = Math.max(0, closest.distance() - MARGIN);
    double maxV = -a * LATENCY + Math.sqrt(a * a * LATENCY * LATENCY + 2 * a * d);

    Line2D wall = closest.line();
    boolean clampX = Math.abs(wall.getX1() - wall.getX2()) < EPS;
    boolean clampY = Math.abs(wall.getY1() - wall.getY2()) < EPS;
    if (clampX) requested.withVelocityX(MathUtil.clamp(requested.VelocityX, -maxV, maxV));
    if (clampY) requested.withVelocityY(MathUtil.clamp(requested.VelocityY, -maxV, maxV));

    SmartDashboard.putBoolean("clamp X", clampX);
    SmartDashboard.putBoolean("clamp Y", clampY);
    SmartDashboard.putNumber("CA maxV", maxV);
    return requested;
}

    private Line2D[] getVectors(double brakeDistance, double angle, Pose2d robotPos, boolean intakeDown) {

        Line2D[] vals = new Line2D[4];
        Translation2d[] offsets;
       
        offsets = new Translation2d[] {
                new Translation2d(robotLength, robotWidth),
                new Translation2d(robotLength, -robotWidth),
                new Translation2d(-robotLength, robotWidth),
                new Translation2d(-robotLength, -robotWidth)
        };
    

        for (int i = 0; i < 4; i++) {
            Translation2d start = robotPos.getTranslation().plus(offsets[i].rotateBy(robotPos.getRotation()));
            Translation2d end = start.plus(new Translation2d(brakeDistance, new Rotation2d(angle)));
            vals[i] = new Line2D.Double(start.getX(), start.getY(), end.getX(), end.getY());
        }

        return vals;
    }

    public record Hit(Line2D line, double distance) {
    }

    // Closest field line crossed by the ray, or null if none
    public static Hit closestHit(Line2D ray) {
        double px = ray.getX1(), py = ray.getY1();
        double rx = ray.getX2() - px, ry = ray.getY2() - py;
        Hit best = null;
        for (Line2D line : FieldLines.field) {
            double sx = line.getX2() - line.getX1(), sy = line.getY2() - line.getY1();
            double denom = rx * sy - ry * sx;
            if (Math.abs(denom) < 1e-12)
                continue; // parallel, never crosses
            double qx = line.getX1() - px, qy = line.getY1() - py;
            double t = (qx * sy - qy * sx) / denom; // fraction along the ray
            double u = (qx * ry - qy * rx) / denom; // fraction along the field line
            if (t >= 0 && t <= 1 && u >= 0 && u <= 1) {
                double dist = t * Math.hypot(rx, ry);
                if (best == null || dist < best.distance())
                    best = new Hit(line, dist);
            }
        }
        return best;
    }
}
