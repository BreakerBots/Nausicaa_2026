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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    /**
     * Pathfind from current pose to the given target pose, avoiding fixed obstacles.
     */
    public Command navigateToPoseCommand(Pose2d target) {
        if (!AutoBuilder.isConfigured()) {
            System.out.println("navigateToPoseCommand: AutoBuilder not configured, skipping pathfind to " + target);
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

    /**
     * Rotates the robot to face the given AprilTag using PID control.
     */
    public Command rotateToTagCommand(int targetTagId) {
        SmartDashboard.putString("Aim/RotateToTagStatus", "Target Tag: " + targetTagId);

        final double toleranceRad = Math.toRadians(0.2);
        final var request = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.Velocity);

        PIDController rotationPID = new PIDController(9, 0.0, 0.1);
        rotationPID.setTolerance(toleranceRad);
        rotationPID.enableContinuousInput(-Math.PI, Math.PI);

        return Commands.run(() -> {
            double angleError = vision.getAngleToTag(targetTagId);
            SmartDashboard.putString("Aim/AngleError", "Angle error:" + angleError);

            double rotationalRate = rotationPID.calculate(0.0, angleError);
            double maxRotRate = Constants.DriveConstants.MAXIMUM_ROTATIONAL_VELOCITY.in(Units.RadiansPerSecond);
            rotationalRate = Math.max(-maxRotRate, Math.min(maxRotRate, rotationalRate));
            drivetrain.setControl(request
                .withVelocityX(0.0)
                .withVelocityY(0.0)
                .withRotationalRate(rotationalRate));
        }, drivetrain)
        .until(() -> {
            if (!vision.isTagDetected(targetTagId)) {
                SmartDashboard.putString("Aim/RotateToTagStatus", "Lost - Target tag: " + targetTagId + " not in view");
                return false;
            }
            double angleError = vision.getAngleToTag(targetTagId);
            boolean areWeThereYet = Math.abs(angleError) <= toleranceRad;
            if (areWeThereYet) {
                SmartDashboard.putString("Aim/RotateToTagStatus", "Done! Final angle remaining: " + String.format("%.3f", Math.toDegrees(angleError)) + " deg");
                return true;
            }
            SmartDashboard.putString("Aim/RotateToTagStatus", "Remaining angle to tag " + targetTagId + ": " + String.format("%.3f", Math.toDegrees(angleError)) + " deg");
            return false;
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
     * While run: driver keeps X/Y; rotation is overridden to face the target AprilTag.
     */
    public Command trackTagCommand(int targetTagId, DoubleSupplier vx, DoubleSupplier vy) {
        final var request = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.Velocity);
        PIDController rotationPID = new PIDController(9, 0.0, 0.1);
        rotationPID.enableContinuousInput(-Math.PI, Math.PI);
        double maxRotRate = Constants.DriveConstants.MAXIMUM_ROTATIONAL_VELOCITY.in(Units.RadiansPerSecond);

        return Commands.run(() -> {
            double angleError = vision.getAngleToTag(targetTagId);
            double omega = rotationPID.calculate(0.0, angleError);
            omega = Math.max(-maxRotRate, Math.min(maxRotRate, omega));
            drivetrain.setControl(request.withVelocityX(vx.getAsDouble()).withVelocityY(vy.getAsDouble()).withRotationalRate(omega));
        }, drivetrain)
        .finallyDo(() -> {
            rotationPID.reset();
            rotationPID.close();
        });
    }

    /**
     * Drives to target distance from the AprilTag. Stops when in tolerance, tag is lost, or timeout.
     */
    public Command rangeToTagCommand(int targetTagId, double targetDistanceMeters) {
        final double toleranceMeters = Constants.DriveConstants.RANGE_TO_TAG_TOLERANCE;
        final double kP = Constants.DriveConstants.RANGE_TO_TAG_KP;
        final double maxVelocity = Constants.DriveConstants.MAXIMUM_TRANSLATIONAL_VELOCITY.in(Units.MetersPerSecond);
        final var request = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.Velocity);

        return Commands.run(() -> {
            Translation2d toTag = vision.getRobotToTagTranslation(targetTagId);
            if (toTag != null) {
                double currentDistanceToTagMeters = toTag.getNorm();
                double remainingDistanceToTargetMeters = currentDistanceToTagMeters - targetDistanceMeters;
                double velocityStraightToTag = Math.max(-maxVelocity, Math.min(maxVelocity, kP * remainingDistanceToTargetMeters));
                double velocityX = 0.0;
                double velocityY = 0.0;
                int allianceFlip = Constants.FieldConstants.getAlliance().get() == Alliance.Red ? -1 : 1;
                if (currentDistanceToTagMeters > 0) {
                    velocityX = (allianceFlip * toTag.getX() / currentDistanceToTagMeters) * velocityStraightToTag;
                    velocityY = (allianceFlip * toTag.getY() / currentDistanceToTagMeters) * velocityStraightToTag;
                }
                drivetrain.setControl(request.withVelocityX(velocityX).withVelocityY(velocityY).withRotationalRate(0.0));
            }
        }, drivetrain)
        .until(() -> {
            Translation2d toTag = vision.getRobotToTagTranslation(targetTagId);
            if (toTag == null) return false;
            double currentDistanceToTagMeters = toTag.getNorm();
            final double requireTagInViewWithinMeters = Constants.DriveConstants.RANGE_TO_TAG_REQUIRE_VISION_WITHIN_METERS;
            if (currentDistanceToTagMeters <= requireTagInViewWithinMeters && !vision.isTagDetected(targetTagId)) {
                return false;
            }
            return currentDistanceToTagMeters >= 0 && Math.abs(currentDistanceToTagMeters - targetDistanceMeters) <= toleranceMeters;
        })
        .withTimeout(5.0)
        .finallyDo((interrupted) -> drivetrain.setControl(request.withVelocityX(0.0).withVelocityY(0.0).withRotationalRate(0.0)));
    }

    /**
     * Rotates to face the tag, then drives to target distance.
     */
    public Command rotateAndRangeToTagCommand(int tagID, double targetDistanceMeters) {
        return Commands.sequence(
            rotateToTagCommand(tagID),
            rangeToTagCommand(tagID, targetDistanceMeters));
    }

    /**
     * Drives to target distance while maintaining heading toward the tag.
     */
    public Command trackAndRangeToTagCommand(int targetTagId, double targetDistanceMeters) {
        final double toleranceMeters = Constants.DriveConstants.RANGE_TO_TAG_TOLERANCE;
        final double kP = Constants.DriveConstants.RANGE_TO_TAG_KP;
        final double maxVelocity = Constants.DriveConstants.MAXIMUM_TRANSLATIONAL_VELOCITY.in(Units.MetersPerSecond);
        final double maxRotRate = Constants.DriveConstants.MAXIMUM_ROTATIONAL_VELOCITY.in(Units.RadiansPerSecond);
        final var request = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.Velocity);

        PIDController rotationPID = new PIDController(9, 0.0, 0.1);
        rotationPID.enableContinuousInput(-Math.PI, Math.PI);

        return Commands.run(() -> {
            Translation2d toTag = vision.getRobotToTagTranslation(targetTagId);
            if (toTag != null) {
                double currentDistanceToTagMeters = toTag.getNorm();
                double remainingDistanceToTargetMeters = currentDistanceToTagMeters - targetDistanceMeters;
                double velocityStraightToTag = Math.max(-maxVelocity, Math.min(maxVelocity, kP * remainingDistanceToTargetMeters));
                double velocityX = 0.0;
                double velocityY = 0.0;
                int allianceFlip = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? -1 : 1;
                if (currentDistanceToTagMeters > 0) {
                    velocityX = (allianceFlip * toTag.getX() / currentDistanceToTagMeters) * velocityStraightToTag;
                    velocityY = (allianceFlip * toTag.getY() / currentDistanceToTagMeters) * velocityStraightToTag;
                }
                double angleError = vision.getAngleToTag(targetTagId);
                double omega = rotationPID.calculate(0.0, angleError);
                omega = Math.max(-maxRotRate, Math.min(maxRotRate, omega));
                drivetrain.setControl(request.withVelocityX(velocityX).withVelocityY(velocityY).withRotationalRate(omega));
            } else {
                drivetrain.setControl(request.withVelocityX(0.0).withVelocityY(0.0).withRotationalRate(0.0));
            }
        }, drivetrain)
        .until(() -> {
            Translation2d toTag = vision.getRobotToTagTranslation(targetTagId);
            if (toTag == null) return false;
            double currentDistanceToTagMeters = toTag.getNorm();
            return currentDistanceToTagMeters >= 0 && Math.abs(currentDistanceToTagMeters - targetDistanceMeters) <= toleranceMeters;
        })
        .withTimeout(5.0)
        .finallyDo((interrupted) -> {
            rotationPID.reset();
            rotationPID.close();
            drivetrain.setControl(request.withVelocityX(0.0).withVelocityY(0.0).withRotationalRate(0.0));
        });
    }
}
