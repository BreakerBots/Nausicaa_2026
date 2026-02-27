package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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

    // Uncomment when experimenting with distance-based hood. Also uncomment DISTANCE_AT_MIN_HOOD_METERS etc. in ShooterConstants, and add: import edu.wpi.first.math.MathUtil;
    // public double getHoodPositionForDistance(double distanceToHubMeters) {
    //     double dMin = Constants.ShooterConstants.DISTANCE_AT_MIN_HOOD_METERS;
    //     double dMax = Constants.ShooterConstants.DISTANCE_AT_MAX_HOOD_METERS;
    //     // Normalize distance to [0, 1]: t=0 at dMin (closest), t=1 at dMax (furthest). Clamp for distances outside range.
    //     double t = (dMax > dMin) ? MathUtil.clamp((distanceToHubMeters - dMin) / (dMax - dMin), 0, 1) : 0;
    //     return MathUtil.interpolate(
    //             Constants.ShooterConstants.HOOD_POSITION_AT_MIN_DISTANCE,
    //             Constants.ShooterConstants.HOOD_POSITION_AT_MAX_DISTANCE,
    //             t);
    // }
}
