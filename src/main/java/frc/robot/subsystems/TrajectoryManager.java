package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Manages trajectory calculations for shooting: distance to target, hood angle from distance/speed.
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
     * Returns the hood position (rotations) for the given target distance and flywheel speed.
     * For now always returns POSITION_HOOD_SETPOINT_2.
     */
    public double getHoodPosition(double targetDistanceMeters, double flywheelSpeed) {
        return Constants.ShooterConstants.POSITION_HOOD_SETPOINT_2;
    }
}
