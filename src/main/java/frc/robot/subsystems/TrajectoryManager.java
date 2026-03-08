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
     * 
     * Hood postition must stay between POSITION_HOOD_MIN and POSITION_HOOD_MAX!
     * 
     * Distance from robot center to chasis front edge = 15.7" (0.39878 meters)
     * 
     * Position Mark + 23.5" 
     * 
     * 12 + 23.5" --> m
     * 
     * 
     */
    private static final Translation2d[] HOOD_DISTANCE_ANGLE_TABLE = {
        new Translation2d(1.2192, 0.02),        //4ft
        new Translation2d(1.524, 0.06),         // 5ft
        new Translation2d(1.8288, 0.115),       // 6 ft
        new Translation2d(2.1336, 0.14),        // 7ft
        new Translation2d(2.4384, 0.172119),    // 8 ft
        new Translation2d(2.7432, 0.212891),    // 9 ft
        new Translation2d(3.05, 0.2459),        //10 ft
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
            return Constants.ShooterConstants.POSITION_HOOD_MIN;
        }
        double dMin = Collections.min(hoodLookup.keySet());
        double dMax = Collections.max(hoodLookup.keySet());
        double clampedDist = MathUtil.clamp(distanceToTargetMeters, dMin, dMax);
        BreakerInterpolableDouble result = hoodLookup.getInterpolatedValue(clampedDist);
        double raw = result != null ? result.getValue() : Constants.ShooterConstants.POSITION_HOOD_MIN;
        return MathUtil.clamp(raw, Constants.ShooterConstants.POSITION_HOOD_MIN, Constants.ShooterConstants.POSITION_HOOD_MAX);
    }
    
}
