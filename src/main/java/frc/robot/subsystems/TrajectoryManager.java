package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.BreakerLib.util.math.BreakerMath;

/**
 * Manages trajectory calculations for shooting: distance to target, hood angle from pose or distance.
 */
public class TrajectoryManager extends SubsystemBase {

    private final Drivetrain drivetrain;

    public TrajectoryManager(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    /** Returns the distance in meters from the robot to the given field point. */
    public double getDistanceToPoint(Translation2d targetPoint) {
        return drivetrain.getRobotToPointTranslation(targetPoint).getNorm();
    }

    /**
     * Returns the predefined hood position (rotations) for the given shooting pose.
     * L1/C1/R1 → POSITION_HOOD_SETPOINT_1, L2/C2/R2/HUB_CENTER → POSITION_HOOD_SETPOINT_2, L3/R3 → POSITION_HOOD_SETPOINT_3.
     */
    public double getHoodPositionForPose(Pose2d pose) {
        if (pose == Constants.FieldConstants.POSE_SHOOTING_BLUE_L1 || 
            pose == Constants.FieldConstants.POSE_SHOOTING_BLUE_C1 || 
            pose == Constants.FieldConstants.POSE_SHOOTING_BLUE_R1 || 
            pose == Constants.FieldConstants.POSE_SHOOTING_RED_L1 || 
            pose == Constants.FieldConstants.POSE_SHOOTING_RED_C1 || 
            pose == Constants.FieldConstants.POSE_SHOOTING_RED_R1) {
            return Constants.ShooterConstants.POSITION_HOOD_SETPOINT_1;
        } else if (
            pose == Constants.FieldConstants.POSE_SHOOTING_BLUE_L3 || 
            pose == Constants.FieldConstants.POSE_SHOOTING_BLUE_R3 ||
            pose == Constants.FieldConstants.POSE_SHOOTING_RED_L3 || 
            pose == Constants.FieldConstants.POSE_SHOOTING_RED_R3) {
            return Constants.ShooterConstants.POSITION_HOOD_SETPOINT_3;
        } else {
            return Constants.ShooterConstants.POSITION_HOOD_SETPOINT_2; 
        }
    }

    /**
     * Returns hood position (encoder rotations) for the given distance from target.
     * Uses Lagrange interpolation through HOOD_DISTANCE_ANGLE_TABLE for a smooth curve.
     * Distance is clamped to the table range to avoid extrapolation.
     */
    public double getHoodPositionForDistance(double distanceToTargetMeters) {
        Translation2d[] table = Constants.ShooterConstants.HOOD_DISTANCE_ANGLE_TABLE;
        if (table == null || table.length == 0) {
            return Constants.ShooterConstants.POSITION_HOOD_SETPOINT_2;
        }
        double dMin = table[0].getX();
        double dMax = table[table.length - 1].getX();
        double clampedDist = MathUtil.clamp(distanceToTargetMeters, dMin, dMax);
        return BreakerMath.interpolateLagrange(clampedDist, table);
    }
}
