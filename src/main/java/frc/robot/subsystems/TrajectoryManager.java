package frc.robot.subsystems;

import java.util.Collections;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.BreakerLib.util.math.interpolation.BreakerInterpolableDouble;
import frc.robot.BreakerLib.util.math.interpolation.maps.BreakerInterpolatingTreeMap;

/**
 * Manages trajectory calculations for shooting: distance to target, hood angle from pose or distance.
 */
public class TrajectoryManager extends SubsystemBase {

    /**
     * Hood position vs distance from hub: (distance m, hood position rotations).
     * Tune through testing – add/remove/adjust pairs as needed.
     */
    private static final Translation2d[] HOOD_DISTANCE_ANGLE_TABLE = {
        new Translation2d(1.5, -0.07),   // close
        new Translation2d(2.0, -0.08),
        new Translation2d(2.5, -0.09),
        new Translation2d(3.0, -0.10),
        new Translation2d(3.5, -0.12),
        new Translation2d(4.0, -0.15),   // far
    };

    private static final BreakerInterpolatingTreeMap<Double, BreakerInterpolableDouble> hoodLookup = buildHoodLookup();

    private final Drivetrain drivetrain;

    public TrajectoryManager(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    private static BreakerInterpolatingTreeMap<Double, BreakerInterpolableDouble> buildHoodLookup() {
        BreakerInterpolatingTreeMap<Double, BreakerInterpolableDouble> map =
            new BreakerInterpolatingTreeMap<>();
        Translation2d[] table = HOOD_DISTANCE_ANGLE_TABLE;
        if (table != null) {
            for (Translation2d pt : table) {
                map.put(pt.getX(), new BreakerInterpolableDouble(pt.getY()));
            }
        }
        return map;
    }

    /** Returns the distance in meters from the robot to the given field point. */
    public double getDistanceToPoint(Translation2d targetPoint) {
        return drivetrain.getRobotToPointTranslation(targetPoint).getNorm();
    }

    /**
     * Returns the predefined hood position (rotations) for the given shooting pose.
     * L1/C1/R1 → POSITION_HOOD_SETPOINT_1, L2/C2/R2/HUB_CENTER → POSITION_HOOD_SETPOINT_2, L3/R3 → POSITION_HOOD_SETPOINT_3.
     */

    // public double getHoodPositionForPose(Pose2d pose) {
    //     if (pose == Constants.FieldConstants.POSE_SHOOTING_BLUE_L1 || 
    //         pose == Constants.FieldConstants.POSE_SHOOTING_BLUE_C1 || 
    //         pose == Constants.FieldConstants.POSE_SHOOTING_BLUE_R1 || 
    //         pose == Constants.FieldConstants.POSE_SHOOTING_RED_L1 || 
    //         pose == Constants.FieldConstants.POSE_SHOOTING_RED_C1 || 
    //         pose == Constants.FieldConstants.POSE_SHOOTING_RED_R1) {
    //         return Constants.ShooterConstants.POSITION_HOOD_SETPOINT_1;
    //     } else if (
    //         pose == Constants.FieldConstants.POSE_SHOOTING_BLUE_L3 || 
    //         pose == Constants.FieldConstants.POSE_SHOOTING_BLUE_R3 ||
    //         pose == Constants.FieldConstants.POSE_SHOOTING_RED_L3 || 
    //         pose == Constants.FieldConstants.POSE_SHOOTING_RED_R3) {
    //         return Constants.ShooterConstants.POSITION_HOOD_SETPOINT_3;
    //     } else {
    //         return Constants.ShooterConstants.POSITION_HOOD_SETPOINT_2; 
    //     }
    // }

    /**
     * Returns hood position in encoder rotations for the given distance to target in meters.
     * Uses linear interpolation through HOOD_DISTANCE_ANGLE_TABLE. Distance is clamped to the
     * table range to avoid extrapolation.
     */
    public static double getHoodPositionForDistance(double distanceToTargetMeters) {
        if (hoodLookup.isEmpty()) {
            return Constants.ShooterConstants.POSITION_HOOD_SETPOINT_2;
        }
        double dMin = Collections.min(hoodLookup.keySet());
        double dMax = Collections.max(hoodLookup.keySet());
        double clampedDist = MathUtil.clamp(distanceToTargetMeters, dMin, dMax);
        BreakerInterpolableDouble result = hoodLookup.getInterpolatedValue(clampedDist);
        return result != null ? result.getValue() : Constants.ShooterConstants.POSITION_HOOD_SETPOINT_2;
    }
}
