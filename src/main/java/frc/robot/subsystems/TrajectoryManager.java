package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Manages trajectory calculations for shooting: distance to target, hood angle from distance.
 */
public class TrajectoryManager extends SubsystemBase {

    private final Drivetrain drivetrain;

    public TrajectoryManager(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    /**
     * Returns the hood position (rotations) for the given distance to hub.
     * Interpolates between HOOD_POSITION_AT_MIN_DISTANCE and HOOD_POSITION_AT_MAX_DISTANCE
     * based on DISTANCE_AT_MIN_HOOD_METERS and DISTANCE_AT_MAX_HOOD_METERS. Tune via Constants.
     */
    public double getHoodPositionForDistance(double distanceToHubMeters) {
        double dMin = Constants.ShooterConstants.DISTANCE_AT_MIN_HOOD_METERS;
        double dMax = Constants.ShooterConstants.DISTANCE_AT_MAX_HOOD_METERS;
        // Normalize distance to [0, 1]: t=0 at dMin (closest), t=1 at dMax (furthest). Clamp for distances outside range.
        double t = (dMax > dMin) ? MathUtil.clamp((distanceToHubMeters - dMin) / (dMax - dMin), 0, 1) : 0;
        return MathUtil.interpolate(
                Constants.ShooterConstants.HOOD_POSITION_AT_MIN_DISTANCE,
                Constants.ShooterConstants.HOOD_POSITION_AT_MAX_DISTANCE,
                t);
    }
}
