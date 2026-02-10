// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.BreakerLib.driverstation.BreakerInputStream;
import frc.robot.BreakerLib.driverstation.BreakerInputStream2d;
import frc.robot.BreakerLib.driverstation.gamepad.controllers.BreakerXboxController;
import frc.robot.BreakerLib.util.logging.BreakerLog;
import frc.robot.BreakerLib.util.math.functions.BreakerLinearizedConstrainedExponential;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Climb;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems , commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    // The robot's subsystems and commands are defined here...
    private final BreakerXboxController controller = new BreakerXboxController(Constants.OperatorConstants.kDriverControllerPort);
    private final Drivetrain drivetrain = new Drivetrain();
    private final Vision vision = new Vision(drivetrain);
    private final Intake intake = new Intake();
    private final Climb climb = new Climb();
    private final Shooter shooter = new Shooter();
    
    private BreakerInputStream driverX, driverY, driverOmega;

    /** PathPlanner auto chooser; populated from GUI autos when AutoBuilder is configured. */
    private final SendableChooser<Command> autoChooser;


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Disable verbose logging to reduce noise
        // Flip this back on when debugging/troubleshooting
        BreakerLog.setVerboseLogging(false);

        // Set up our auto-chooser    
        if (AutoBuilder.isConfigured()) {
            // Looks for autos in /src/main/deploy/pathplanner/autos/
            autoChooser = AutoBuilder.buildAutoChooser();
        } else {
            autoChooser = new SendableChooser<>();
            autoChooser.setDefaultOption("Do Nothing", Commands.none());
        }
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // Bind our controller buttons
        configureBindings();
    }


    /**
     * Use this method to define your trigger->command mappings. 
     */
    private void configureBindings() {

        // LEFT BUMPER --> RESET LOCALIZER'S POSE
        controller.getLeftBumper().onTrue(Commands.runOnce(() -> drivetrain.getLocalizer().resetPose(new Pose2d(0,0, Rotation2d.fromRotations(0.0)))));

        // RIGHT BUMPER --> NAVIGATE FROM CURRENT POSE TO TARGET POSE (PathPlanner)
        //controller.getRightBumper().onTrue(navigateToPoseCommand(Constants.FieldConstants.POSE_RED_TRENCH_IN_RED_AZ));


        // ---------------- SWERVE DRIVE ----------------

        // LEFT THUMBSTICK --> DRIVE
        BreakerInputStream2d driverTranslation = controller.getLeftThumbstick();
        driverTranslation = driverTranslation
                .clamp(1.0)
                .deadband(Constants.OperatorConstants.TRANSLATIONAL_DEADBAND, 1.0)
                .mapToMagnitude(new BreakerLinearizedConstrainedExponential(0.075, 3.0, true))
                .scale(Constants.DriveConstants.MAXIMUM_TRANSLATIONAL_VELOCITY.in(Units.MetersPerSecond));
        driverX = driverTranslation.getY();
        driverY = driverTranslation.getX();

        // RIGHT THUMBSTICK --> ROTATE
        driverOmega = controller.getRightThumbstick().getX()
                .clamp(1.0)
                .deadband(Constants.OperatorConstants.ROTATIONAL_DEADBAND, 1.0)
                .map(new BreakerLinearizedConstrainedExponential(0.364, 6.6, true))
                .scale(Constants.DriveConstants.MAXIMUM_ROTATIONAL_VELOCITY.in(Units.RadiansPerSecond));
    
        drivetrain.setDefaultCommand(drivetrain.getTeleopControlCommand(driverX, driverY, driverOmega, Constants.DriveConstants.TELEOP_CONTROL_CONFIG));

        // RIGHT TRIGGER (held) --> TRACK TAG; driver keeps X/Y control, rotation follows tag
        controller.getRightTrigger().whileTrue(Commands.runOnce(() -> {
            CommandScheduler.getInstance().schedule(trackTagCommand(Constants.FieldConstants.getHubTagID()));
        }));

        // A button (pressed) --> Range to tag
        controller.getButtonA().onTrue(Commands.runOnce(() -> {
            CommandScheduler.getInstance().schedule(rangeToTag2Command(Constants.FieldConstants.getHubTagID(), 1.0));
        }));

        // ----------------- INTAKE -------------
        
        // B: EXTENDED_INTAKING ↔ EXTENDED_IDLE (toggle: if not intaking → intaking; if intaking → idle)
        controller.getButtonB().onTrue(Commands.runOnce(() -> {
            SmartDashboard.putString("Intake/ButtonStatus", "Setting state...");
            if (intake.state == Intake.State.EXTENDED_IDLE) {
                SmartDashboard.putString("Intake/ButtonStatus", "Setting state to stowed");
                intake.setState(Intake.State.STOWED);
            } else {
                SmartDashboard.putString("Intake/ButtonStatus", "Setting state to extended-idle");
                intake.setState(Intake.State.EXTENDED_IDLE);
            }
        }, intake));



        // ----------------- HOPPER/FEEDER -------------

        // A: ?


        // ----------------- SHOOTER -------------

        // X: Rotate to face hub tag, then range to target distance (alliance-aware)
        controller.getButtonX().onTrue(Commands.runOnce(() -> {
            CommandScheduler.getInstance().schedule(rotateAndRangeToTagCommand(Constants.FieldConstants.getHubTagID(), 1.0));
        }));

        // Y: Toggle shooter state (INACTIVE ↔ SHOOTING)
        
        // //INACTIVE
        // controller.getDPad().getDown().onTrue(shooter.setStateCommand(Shooter.State.INACTIVE));

        // //SHOOTING
        // controller.getDPad().getUp().onTrue(shooter.setStateCommand(Shooter.State.SHOOTING));


        // ----------------- CLIMB -------------

        // D-PAD UP --> CLIMB UP
        controller.getDPad().getUp().onTrue(climb.climbToUpCommand());
        // D-PAD DOWN --> CLIMB DOWN    
        controller.getDPad().getDown().onTrue(climb.climbToDownCommand());
    }


    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public Drivetrain getDrivetrain() {
        return drivetrain;
    }

    /**
     * Pathfind from current pose to the given target pose,
     * avoiding fixed obstacles using PathPlanner navgrid (deploy/pathplanner/navgrid.json).
     */
    public Command navigateToPoseCommand(Pose2d target) {
        if (!AutoBuilder.isConfigured()) {
            System.out.println("navigateToPoseCommand: AutoBuilder not configured, skipping pathfind to " + target);
            return Commands.none();
        }
        PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // Minimal constraints for now
        return AutoBuilder.pathfindToPose(
            target,
            constraints,
            0.0 
        );
    }

    /**
     * Rotates to face the given AprilTag, then drives to the target distance from it.
     * Uses {@link #rotateToTagCommand(int)} then {@link #rangeToTagCommand(int, double)}.
     */
    private Command rotateAndRangeToTagCommand(int tagID, double targetDistanceMeters) {
        return Commands.sequence(
            rotateToTagCommand(tagID),
            rangeToTagCommand(tagID, targetDistanceMeters));
    }

    /**
     * Rotates the robot to face the given AprilTag using PID control.
     */
    private Command rotateToTagCommand(int targetTagId) {

        System.out.println("rotateToTag: Target AprilTag: " + targetTagId);
        SmartDashboard.putString("Aim/RotateToTagStatus", "Target Tag: " + targetTagId);

        final double toleranceRad = Math.toRadians(0.2); // degree
        final var request = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.Velocity);

        PIDController rotationPID = new PIDController(9, 0.0, 0.1);
        rotationPID.setTolerance(toleranceRad);
        rotationPID.enableContinuousInput(-Math.PI, Math.PI); // Handle wrap-around

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
            if (!vision.isTagDetected()) {
                System.err.println("rotateToTag: lost target tag " + targetTagId + " (no tag in view)");
                SmartDashboard.putString("Aim/RotateToTagStatus", "Lost - No tag in view");
                return true; // Stop and give control back
            }
            if (vision.getNearestDetectedTagId() != targetTagId) {
                System.err.println("rotateToTag: lost target tag " + targetTagId + " (target no longer in view)");
                SmartDashboard.putString("Aim/RotateToTagStatus", "Lost - Target tag: " + targetTagId + " not in view");
                return true; // Stop and give control back
            }
            double angleError = vision.getAngleToTag(targetTagId);
            SmartDashboard.putString("Aim/RotateToTagStatus", "Remaining angle to tag: " + targetTagId + ": " + Math.toDegrees(angleError));
            return Math.abs(angleError) <= toleranceRad; // Aligned, done   
        })

        .withTimeout(3.0) // Always end so default drive command can run again
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
     * Ranges the robot to a distance from the given AprilTag using PID control.
     */
    private Command rangeToTagCommand(int targetTagId, double targetDistanceMeters) {

        // System.out.println("rotateToTag: Target AprilTag: " + targetTagId);
        // SmartDashboard.putString("Aim/RotateToTagStatus", "Target Tag: " + targetTagId);

        final double toleranceMeters = 0.01; // distance
        final var request = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.Velocity);

        PIDController translationalPID = new PIDController(9, 0.0, 0.1);
        translationalPID.setTolerance(toleranceMeters);
        // rotationalPID.enableContinuousInput(-Math.PI, Math.PI); // Handle wrap-around

        return Commands.run(() -> {
            double distanceError = vision.getDistanceToTag(targetTagId) - targetDistanceMeters;
            double distanceErrorX = drivetrain.getLocalizer().getPose().getRotation().getCos() * distanceError;
            double distanceErrorY = drivetrain.getLocalizer().getPose().getRotation().getSin() * distanceError;
            SmartDashboard.putString("Aim/DistanceErrorX", "Distance error (X):" + distanceErrorX);
            SmartDashboard.putString("Aim/DistanceErrorY", "Distance error (Y):" + distanceErrorY);
            SmartDashboard.putString("Aim/DistanceError", "Distance error (total):" + distanceError);

            double translationalRateX = translationalPID.calculate(0.0, distanceErrorX);
            double translationalRateY = translationalPID.calculate(0.0, distanceErrorY);
            double maxVelocity = Constants.DriveConstants.MAXIMUM_TRANSLATIONAL_VELOCITY.in(Units.MetersPerSecond);
            translationalRateX = Math.max(-maxVelocity, Math.min(maxVelocity, translationalRateX));
            translationalRateY = Math.max(-maxVelocity, Math.min(maxVelocity, translationalRateY));
            drivetrain.setControl(request
                .withVelocityX(translationalRateX)
                .withVelocityY(translationalRateY)
                .withRotationalRate(0.0));
        }, drivetrain)
        .until(() -> {
            if (!vision.isTagDetected()) {
                System.err.println("rangeToTag: lost target tag " + targetTagId + " (no tag in view)");
                SmartDashboard.putString("Aim/DistanceToTagStatus", "Lost - No tag in view");
                return true; // Stop and give control back
            }
            if (vision.getNearestDetectedTagId() != targetTagId) {
                System.err.println("rangeToTag: lost target tag " + targetTagId + " (target no longer in view)");
                SmartDashboard.putString("Aim/DistanceToTagStatus", "Lost - Target tag: " + targetTagId + " not in view");
                return true; // Stop and give control back
            }
            double distanceError = vision.getDistanceToTag(targetTagId) - targetDistanceMeters;
            SmartDashboard.putString("Aim/RangeToTagStatus", "Remaining distance to tag: " + targetTagId + ": " + distanceError);
            return Math.abs(distanceError) <= toleranceMeters; // Ranged, done   
        })

        .withTimeout(3.0) // Always end so default drive command can run again
        .finallyDo(() -> {
            translationalPID.reset();
            translationalPID.close();
            drivetrain.setControl(request
                .withVelocityX(0.0)
                .withVelocityY(0.0)
                .withRotationalRate(0.0));
        });
    }

    /**
     * Drives the robot to a target distance (m) from the AprilTag. Uses field-centric X/Y so the
     * robot moves straight toward or away from the tag regardless of heading. 
     * Stops when in tolerance, tag is lost, or timeout.
     */
    private Command rangeToTag2Command(int targetTagId, double targetDistanceMeters) {
        final double toleranceMeters = Constants.DriveConstants.RANGE_TO_TAG_TOLERANCE; // How close to the target is "close enough"?
        final double kP = Constants.AutoConstants.PATHPLANNER_TRANSLATION_PID.kP; // Proportional Gain: how fast to move toward/away from the tag?
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
                if (currentDistanceToTagMeters > 0) {
                    velocityX = (toTag.getX() / currentDistanceToTagMeters) * velocityStraightToTag;
                    velocityY = (toTag.getY() / currentDistanceToTagMeters) * velocityStraightToTag;
                }
                drivetrain.setControl(request.withVelocityX(velocityX).withVelocityY(velocityY).withRotationalRate(0.0));
            }
        }, drivetrain)
        .until(() -> {
            if (!vision.isTagDetected() || vision.getNearestDetectedTagId() != targetTagId) {
                return true; // If we lost our tag, bail out
            }
            double currentDistanceToTagMeters = vision.getDistanceToTag(targetTagId);
            boolean areWeThereYet = currentDistanceToTagMeters >= 0 && Math.abs(currentDistanceToTagMeters - targetDistanceMeters) <= toleranceMeters;
            if (areWeThereYet == true) {
                SmartDashboard.putString("Aim/RangeToTagStatus", "Done! At the target distance of " + targetDistanceMeters + "m");
            } else {
                SmartDashboard.putString("Aim/RangeToTagStatus", "We're " + currentDistanceToTagMeters + "m from the target");
            }
            return areWeThereYet;
        })
        .withTimeout(5.0)
        .finallyDo(() -> drivetrain.setControl(request
            .withVelocityX(0.0).withVelocityY(0.0).withRotationalRate(0.0)));
    }

    /**
     * While run: driver keeps X/Y (forward and strafe); rotation is overridden to face the target AprilTag.
     * Use with trigger .whileTrue() so holding trigger = track tag.
     */
    private Command trackTagCommand(int targetTagId) {
        final var request = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.Velocity);
        PIDController rotationPID = new PIDController(9, 0.0, 0.1);
        rotationPID.enableContinuousInput(-Math.PI, Math.PI);
        double maxRotRate = Constants.DriveConstants.MAXIMUM_ROTATIONAL_VELOCITY.in(Units.RadiansPerSecond);

        return Commands.run(() -> {
            double vx = driverX.get();
            double vy = driverY.get();
            double angleError = vision.getAngleToTag(targetTagId);
            double omega = rotationPID.calculate(0.0, angleError);
            omega = Math.max(-maxRotRate, Math.min(maxRotRate, omega));
            drivetrain.setControl(request.withVelocityX(vx).withVelocityY(vy).withRotationalRate(omega));
        }, drivetrain)
        .finallyDo(() -> {
            rotationPID.reset();
        });
    }

}