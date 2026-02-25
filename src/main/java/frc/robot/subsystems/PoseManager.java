package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

/**
 * Manages commands for changing the robot's pose on the field: pathfinding, aligning to tags,
 * tracking tags, and ranging to tags.
 */
public class PoseManager extends SubsystemBase {

    private final Drivetrain drivetrain;
    private final Vision vision;

    public PoseManager(Drivetrain drivetrain, Vision vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;
    }

    // --------------- PATHPLANNER ON-THE-FLY ---------------

    /**
     * Pathfind from current pose to the given target pose, avoiding fixed obstacles.
     * Refuses to run if robot is farther than NAVIGATE_TO_POSE_MAX_DISTANCE_METERS from target
     * (intended for short finishing moves, not long-distance drives).
     */
    public Command navigateToPoseCommand(Pose2d target) {
        if (!AutoBuilder.isConfigured()) {
            System.out.println("navigateToPoseCommand: AutoBuilder not configured, skipping pathfind to " + target);
            return Commands.none();
        }
        double distanceMeters = drivetrain.getLocalizer().getPose().getTranslation().getDistance(target.getTranslation());
        if (distanceMeters > Constants.DriveConstants.NAVIGATE_TO_POSE_MAX_DISTANCE_METERS) {
            System.out.println("navigateToPoseCommand: Robot " + String.format("%.1f", distanceMeters)
                    + " m from target (max " + Constants.DriveConstants.NAVIGATE_TO_POSE_MAX_DISTANCE_METERS + " m), skipping");
            return Commands.none();
        }
        PathConstraints constraints = new PathConstraints(
            Constants.DriveConstants.MAXIMUM_TRANSLATIONAL_VELOCITY.magnitude(),
            1000000000.0,
            Constants.DriveConstants.MAXIMUM_ROTATIONAL_VELOCITY.magnitude(),
            1000000000.0,
            12.0,
            false);
        return AutoBuilder.pathfindToPose(target, constraints, 0.0);
    }

    // --------------- ROTATE TO ---------------

    /**
     * Rotates the robot to face a point on the field using odometry (no vision required).
     * Stops when heading is within tolerance or timeout.
     */
    public Command rotateToPointCommand(Translation2d targetPoint) {
        final double toleranceRad = Math.toRadians(2.0);
        final var request = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.Velocity);

        PIDController rotationPID = new PIDController(9, 0.0, 0.1);
        rotationPID.setTolerance(toleranceRad);
        rotationPID.enableContinuousInput(-Math.PI, Math.PI);

        return Commands.run(() -> {
            Translation2d toTarget = drivetrain.getRobotToPointTranslation(targetPoint);
            double desiredHeading = Math.atan2(toTarget.getY(), toTarget.getX());
            double currentHeading = drivetrain.getLocalizer().getPose().getRotation().getRadians();
            double angleError = Math.IEEEremainder(desiredHeading - currentHeading, 2.0 * Math.PI);

            double rotationalRate = rotationPID.calculate(0.0, angleError);
            double maxRotRate = Constants.DriveConstants.MAXIMUM_ROTATIONAL_VELOCITY.in(Units.RadiansPerSecond);
            rotationalRate = Math.max(-maxRotRate, Math.min(maxRotRate, rotationalRate));

            drivetrain.setControl(request
                .withVelocityX(0.0)
                .withVelocityY(0.0)
                .withRotationalRate(rotationalRate));
        }, drivetrain)
        .until(() -> {
            Translation2d toTarget = drivetrain.getRobotToPointTranslation(targetPoint);
            double desiredHeading = Math.atan2(toTarget.getY(), toTarget.getX());
            double currentHeading = drivetrain.getLocalizer().getPose().getRotation().getRadians();
            double angleError = Math.IEEEremainder(desiredHeading - currentHeading, 2.0 * Math.PI);
            return Math.abs(angleError) <= toleranceRad;
        })
        .withTimeout(3.0)
        .finallyDo(() -> {
            rotationPID.reset();
            rotationPID.close();
            drivetrain.setControl(request
                .withVelocityX(0.0)
                .withVelocityY(0.0)
                .withRotationalRate(0.0));
        });
    }

    /**
     * Rotates the robot to face the hub center (alliance-aware). Uses odometry, no vision required.
     */
    public Command rotateToHubCenterCommand() {
        return Commands.sequence(
                Commands.runOnce(() -> drivetrain.getLocalizer().resetPose(drivetrain.getLocalizer().getPose()), drivetrain),
                rotateToPointCommand(Constants.FieldConstants.getTargetHubCenter()));
    }

    /**
     * Rotates the robot to face the given AprilTag.
     * Uses the field layout to get the tag's position, then delegates to rotateToPointCommand.
     * Works without vision (odometry-based); tag need not be visible.
     */
    public Command rotateToTagCommand(int targetTagId) {
        Translation2d tagPosition = vision.getTagPosition(targetTagId);
        if (tagPosition == null) {
            return Commands.none(); // Tag not in field layout
        }
        return rotateToPointCommand(tagPosition);
    }

    // --------------- MAINTAIN HEADING TO ---------------

    /**
     * While run: driver keeps X/Y; rotation is overridden to face the target point (odometry-based).
     */
    public Command trackPointCommand(Translation2d targetPoint, DoubleSupplier vx, DoubleSupplier vy) {
        final var request = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.Velocity);
        PIDController rotationPID = new PIDController(9, 0.0, 0.1);
        rotationPID.enableContinuousInput(-Math.PI, Math.PI);
        double maxRotRate = Constants.DriveConstants.MAXIMUM_ROTATIONAL_VELOCITY.in(Units.RadiansPerSecond);

        return Commands.run(() -> {
            Translation2d toTarget = drivetrain.getRobotToPointTranslation(targetPoint);
            double desiredHeading = Math.atan2(toTarget.getY(), toTarget.getX());
            double currentHeading = drivetrain.getLocalizer().getPose().getRotation().getRadians();
            double angleError = Math.IEEEremainder(desiredHeading - currentHeading, 2.0 * Math.PI);

            double omega = rotationPID.calculate(0.0, angleError);
            omega = Math.max(-maxRotRate, Math.min(maxRotRate, omega));
            drivetrain.setControl(request
                .withVelocityX(vx.getAsDouble())
                .withVelocityY(vy.getAsDouble())
                .withRotationalRate(omega));
        }, drivetrain)
        .finallyDo(() -> {
            rotationPID.reset();
            rotationPID.close();
        });
    }

    /**
     * While run: driver keeps X/Y; rotation is overridden to face the target AprilTag.
     * Uses field layout for tag position, delegates to trackPointCommand.
     */
    public Command trackTagCommand(int targetTagId, DoubleSupplier vx, DoubleSupplier vy) {
        Translation2d tagPosition = vision.getTagPosition(targetTagId);
        if (tagPosition == null) {
            return Commands.none(); // Tag not in field layout
        }
        return trackPointCommand(tagPosition, vx, vy);
    }

    // --------------- RANGE TO ---------------

    /**
     * Drives to target distance from a point on the field (odometry-based). Stops when in tolerance or timeout.
     */
    public Command rangeToPointCommand(Translation2d targetPoint, double targetDistanceMeters) {
        final double toleranceMeters = Constants.DriveConstants.RANGE_TO_TARGET_TOLERANCE;
        final double kP = Constants.DriveConstants.RANGE_TO_TARGET_KP;
        final double maxVelocity = Constants.DriveConstants.MAXIMUM_TRANSLATIONAL_VELOCITY.in(Units.MetersPerSecond);
        final var request = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.Velocity);

        return Commands.run(() -> {
            Translation2d toTarget = drivetrain.getRobotToPointTranslation(targetPoint);
            double currentDistanceMeters = toTarget.getNorm();
            double remainingMeters = currentDistanceMeters - targetDistanceMeters;
            double velocityMagnitude = Math.max(-maxVelocity, Math.min(maxVelocity, kP * remainingMeters));

            double velocityX;
            double velocityY;
            if (currentDistanceMeters > 1e-6) {
                velocityX = (toTarget.getX() / currentDistanceMeters) * velocityMagnitude;
                velocityY = (toTarget.getY() / currentDistanceMeters) * velocityMagnitude;
            } else {
                // At target: drive in robot's heading to escape (direction doesn't matter when distance is 0)
                double heading = drivetrain.getLocalizer().getPose().getRotation().getRadians();
                velocityX = Math.cos(heading) * velocityMagnitude;
                velocityY = Math.sin(heading) * velocityMagnitude;
            }
            drivetrain.setControl(request.withVelocityX(velocityX).withVelocityY(velocityY).withRotationalRate(0.0));
        }, drivetrain)
        .until(() -> {
            double currentDistanceMeters = drivetrain.getRobotToPointTranslation(targetPoint).getNorm();
            return Math.abs(currentDistanceMeters - targetDistanceMeters) <= toleranceMeters;
        })
        .withTimeout(5.0)
        .finallyDo((interrupted) -> drivetrain.setControl(request.withVelocityX(0.0).withVelocityY(0.0).withRotationalRate(0.0)));
    }

    /**
     * Drives to target distance from the AprilTag. Uses field layout for tag position, delegates to rangeToPointCommand.
     */
    public Command rangeToTagCommand(int targetTagId, double targetDistanceMeters) {
        Translation2d tagPosition = vision.getTagPosition(targetTagId);
        if (tagPosition == null) {
            return Commands.none(); // Tag not in field layout
        }
        return rangeToPointCommand(tagPosition, targetDistanceMeters);
    }

    // --------------- COMBINATIONS: ROTATE AND RANGE ---------------

    /**
     * Drives to target distance from a point while maintaining heading toward the point.
     */
    public Command trackAndRangeToPointCommand(Translation2d targetPoint, double targetDistanceMeters) {
        final double toleranceMeters = Constants.DriveConstants.RANGE_TO_TARGET_TOLERANCE;
        final double kP = Constants.DriveConstants.RANGE_TO_TARGET_KP;
        final double maxVelocity = Constants.DriveConstants.MAXIMUM_TRANSLATIONAL_VELOCITY.in(Units.MetersPerSecond);
        final double maxRotRate = Constants.DriveConstants.MAXIMUM_ROTATIONAL_VELOCITY.in(Units.RadiansPerSecond);
        final var request = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.Velocity);

        PIDController rotationPID = new PIDController(9, 0.0, 0.1);
        rotationPID.enableContinuousInput(-Math.PI, Math.PI);

        return Commands.run(() -> {
            Pose2d pose = drivetrain.getLocalizer().getPose();
            Translation2d toTarget = drivetrain.getRobotToPointTranslation(targetPoint);
            double currentDistanceMeters = toTarget.getNorm();
            double remainingMeters = currentDistanceMeters - targetDistanceMeters;
            double velocityMagnitude = Math.max(-maxVelocity, Math.min(maxVelocity, kP * remainingMeters));

            double velocityX;
            double velocityY;
            double omega;
            if (currentDistanceMeters > 1e-6) {
                velocityX = (toTarget.getX() / currentDistanceMeters) * velocityMagnitude;
                velocityY = (toTarget.getY() / currentDistanceMeters) * velocityMagnitude;
                double desiredHeading = Math.atan2(toTarget.getY(), toTarget.getX());
                double currentHeading = pose.getRotation().getRadians();
                double angleError = Math.IEEEremainder(desiredHeading - currentHeading, 2.0 * Math.PI);
                omega = rotationPID.calculate(0.0, angleError);
            } else {
                // At target: drive in robot's heading to escape
                double heading = pose.getRotation().getRadians();
                velocityX = Math.cos(heading) * velocityMagnitude;
                velocityY = Math.sin(heading) * velocityMagnitude;
                omega = 0.0;
            }
            omega = Math.max(-maxRotRate, Math.min(maxRotRate, omega));
            drivetrain.setControl(request.withVelocityX(velocityX).withVelocityY(velocityY).withRotationalRate(omega));
        }, drivetrain)
        .until(() -> {
            double currentDistanceMeters = drivetrain.getRobotToPointTranslation(targetPoint).getNorm();
            return Math.abs(currentDistanceMeters - targetDistanceMeters) <= toleranceMeters;
        })
        .withTimeout(5.0)
        .finallyDo((interrupted) -> {
            rotationPID.reset();
            rotationPID.close();
            drivetrain.setControl(request.withVelocityX(0.0).withVelocityY(0.0).withRotationalRate(0.0));
        });
    }

    /**
     * Drives to target distance from the AprilTag while maintaining heading toward the tag.
     */
    public Command trackAndRangeToTagCommand(int targetTagId, double targetDistanceMeters) {
        Translation2d tagPosition = vision.getTagPosition(targetTagId);
        if (tagPosition == null) {
            return Commands.none(); // Tag not in field layout
        }
        return trackAndRangeToPointCommand(tagPosition, targetDistanceMeters);
    }

    /**
     * Rotates to face the tag, then drives to target distance.
     */
    public Command rotateAndRangeToTagCommand(int targetTagId, double targetDistanceMeters) {
        return Commands.sequence(
            rotateToTagCommand(targetTagId),
            rangeToTagCommand(targetTagId, targetDistanceMeters));
    }
}
