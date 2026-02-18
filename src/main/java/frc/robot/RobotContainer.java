// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;

import java.util.Set;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

    private boolean slowMode;

    /** PathPlanner auto chooser; populated from GUI autos when AutoBuilder is configured. */
    private final SendableChooser<Command> autoChooser;


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Disable verbose logging to reduce noise
        // Flip this back on when debugging/troubleshooting
        BreakerLog.setVerboseLogging(false);

        // Register named commands for PathPlanner event markers (must be before buildAutoChooser)
        NamedCommands.registerCommand("rotateToHub", Commands.defer(() -> rotateToTagCommand(Constants.FieldConstants.getHubTagID()), Set.of(drivetrain)));
        NamedCommands.registerCommand("enterSlowMode", Commands.defer(() -> Commands.runOnce(() -> slowMode = !slowMode), Set.of(drivetrain)));
        NamedCommands.registerCommand("consolidatePose", Commands.defer(() -> Commands.runOnce(() -> drivetrain.getLocalizer().resetPose(new Pose2d(0,0, Rotation2d.fromRotations(0.0)))), Set.of(drivetrain)));


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

        // BACK BUTTON --> SLOW MODE
        controller.getBackButton().onTrue(Commands.runOnce(() -> slowMode = !slowMode));

        // LEFT BUMPER --> RESET LOCALIZER'S POSE
        controller.getLeftBumper().onTrue(Commands.runOnce(() -> drivetrain.getLocalizer().resetPose(new Pose2d(0,0, Rotation2d.fromRotations(0.0)))));

        // RIGHT BUMPER --> Pathfind from current position to POSE_NAVIGATE_TARGET (PathPlanner on-the-fly)
        // controller.getRightBumper().whileTrue(
        //     Commands.runOnce(() -> controller.getLeftBumper().onTrue(navigateToPoseCommand(Constants.FieldConstants.POSE_BLUE_HUB_CENTER)))
        //     );

        controller.getButtonX().and(controller.getRightBumper()).onTrue(navigateToPoseCommand(Constants.FieldConstants.POSE_BLUE_HUB_CENTER));


        // ---------------- SWERVE DRIVE ----------------

        // "Slow Mode" versus Normal
        DoubleSupplier translationalScale = () -> {
            return slowMode ? 
                Constants.DriveConstants.ALIGN_MODE_MAXIMUM_TRANSLATIONAL_VELOCITY.in(Units.MetersPerSecond) : 
                Constants.DriveConstants.MAXIMUM_TRANSLATIONAL_VELOCITY.in(Units.MetersPerSecond);
        };
        DoubleSupplier rotationalScale = () -> {
            return slowMode ? 
                Constants.DriveConstants.ALIGN_MODE_MAXIMUM_ROTATIONAL_VELOCITY.in(Units.RadiansPerSecond) : 
                Constants.DriveConstants.MAXIMUM_ROTATIONAL_VELOCITY.in(Units.RadiansPerSecond);
        };

        // LEFT THUMBSTICK --> DRIVE
        BreakerInputStream2d driverTranslation = controller.getLeftThumbstick();
        driverTranslation = driverTranslation
                .clamp(1.0)
                .deadband(Constants.OperatorConstants.TRANSLATIONAL_DEADBAND, 1.0)
                .mapToMagnitude(new BreakerLinearizedConstrainedExponential(0.075, 3.0, true))
                .scale(translationalScale);
        driverX = driverTranslation.getY();
        driverY = driverTranslation.getX();


        // RIGHT THUMBSTICK --> ROTATE
        driverOmega = controller.getRightThumbstick().getX()
                .clamp(1.0)
                .deadband(Constants.OperatorConstants.ROTATIONAL_DEADBAND, 1.0)
                .map(new BreakerLinearizedConstrainedExponential(0.364, 6.6, true))
                .scale(rotationalScale);
    
        drivetrain.setDefaultCommand(drivetrain.getTeleopControlCommand(driverX, driverY, driverOmega, Constants.DriveConstants.TELEOP_CONTROL_CONFIG));

        // RIGHT TRIGGER (held) --> TRACK TAG; driver keeps X/Y control, rotation follows tag
        controller.getRightTrigger().whileTrue(trackTagCommand(Constants.FieldConstants.getHubTagID()));

        // A button (pressed) --> Range to tag
        controller.getButtonA().onTrue(Commands.runOnce(() -> {
            CommandScheduler.getInstance().schedule(rangeToTagCommand(Constants.FieldConstants.getTrenchTagID(), 1.0));
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
        //controller.getButtonX().onTrue(Commands.runOnce(() -> {
        //    CommandScheduler.getInstance().schedule(rotateAndRangeToTagCommand(Constants.FieldConstants.getHubTagID(), 1.0));
        //}));

        // X button (pressed) --> Range to tag
        controller.getButtonX().and(controller.getRightBumper().negate()).onTrue(Commands.runOnce(() -> {
            CommandScheduler.getInstance().schedule(rotateToTagCommand(Constants.FieldConstants.getHubTagID()));
        }));

        controller.getButtonY().onTrue(trackAndRangeToTagCommand(Constants.FieldConstants.getHubTagID(), 1.0)); // Was rotateAndRangeToTagCommand

        // Y: Toggle shooter state (INACTIVE ↔ SHOOTING)
        
        // //INACTIVE
        // controller.getDPad().getDown().onTrue(shooter.setStateCommand(Shooter.State.INACTIVE));

        // //SHOOTING
        // controller.getDPad().getUp().onTrue(shooter.setStateCommand(Shooter.State.SHOOTING));


        // ----------------- CLIMB -------------

        // LEFT TRIGGER --> Extend (if retracted/retracting) or Retract (if extended/extending).
        // Pressing again during an in-progress extend cancels it and retracts (and vice versa).
        controller.getLeftTrigger().onTrue(Commands.runOnce(() -> {
            if (climb.isRetractedOrRetracting()) {
                CommandScheduler.getInstance().schedule(climb.extend());
            } else if (climb.isExtendedOrExtending()) {
                CommandScheduler.getInstance().schedule(climb.retract());
            }
        }));

        // // D-PAD UP (held) --> Extend manually; release to stop.
        controller.getDPad().getUp().onTrue(Commands.runOnce(() -> {
            CommandScheduler.getInstance().schedule(climb.extend());
        }
        ));
        // // D-PAD DOWN (held) --> Retract manually; release to stop.
        controller.getDPad().getDown().onTrue(Commands.runOnce(() -> {
            CommandScheduler.getInstance().schedule(climb.retract());
        }
        ));
    }


    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public Drivetrain getDrivetrain() {
        return drivetrain;
    }

    /** Called once when the robot enters autonomous. */
    public void autonomousInit() {
        intake.zeroEncoders();
        intake.setState(Intake.State.STOWED);
        climb.zeroEncoder();
        climb.setState(Climb.State.INACTIVE);
    }

    /** Called once when the robot enters teleop. */
    public void teleopInit() {
        intake.zeroEncoders();
        intake.setState(Intake.State.STOWED);
        climb.zeroEncoder();
        climb.setState(Climb.State.INACTIVE);
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
        // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // Minimal constraints for now
        PathConstraints constraints = new PathConstraints(
            Constants.DriveConstants.MAXIMUM_TRANSLATIONAL_VELOCITY.magnitude(), 
            1000000000.0, 
            Constants.DriveConstants.MAXIMUM_ROTATIONAL_VELOCITY.magnitude(), 
            1000000000.0, 
            12.0, 
            false
            );
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
        System.out.println("Sequencing commands...");
        return Commands.sequence(
            rotateToTagCommand(tagID),
            rangeToTagCommand(tagID, targetDistanceMeters));
    }

    /**
     * Drives to target distance from the AprilTag while actively maintaining heading toward the tag
     * (like holding trackTag). Combines range-to-tag translation with rotation-to-face-tag.
     * Stops when within distance tolerance or timeout.
     */
    private Command trackAndRangeToTagCommand(int targetTagId, double targetDistanceMeters) {
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
            System.out.println("rotateToTag: Angle error: " + Math.toDegrees(angleError));
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
                //return true; // Stop and give control back
            }
            if (!vision.isTagDetected(targetTagId)) {
                System.err.println("rotateToTag: lost target tag " + targetTagId + " (target no longer in view)");
                SmartDashboard.putString("Aim/RotateToTagStatus", "Lost - Target tag: " + targetTagId + " not in view");
                //return true; // Stop and give control back
            }
            double angleError = vision.getAngleToTag(targetTagId);
            boolean areWeThereYet = Math.abs(angleError) <= toleranceRad;
            if (areWeThereYet) {
                System.out.println("rotateToTag: Done! Final angle remaining: " + String.format("%.3f", Math.toDegrees(angleError)) + " deg");
                SmartDashboard.putString("Aim/RotateToTagStatus", "Done! Final angle remaining: " + String.format("%.3f", Math.toDegrees(angleError)) + " deg");
                return true;
            }
            System.out.println("rotateToTag: Remaining angle to tag " + targetTagId + ": " + String.format("%.3f", Math.toDegrees(angleError)) + " deg");
            SmartDashboard.putString("Aim/RotateToTagStatus", "Remaining angle to tag " + targetTagId + ": " + String.format("%.3f", Math.toDegrees(angleError)) + " deg");
            return false;
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
     * Drives the robot to a target distance (m) from the AprilTag. Uses field-centric X/Y so the
     * robot moves straight toward or away from the tag regardless of heading. 
     * Stops when in tolerance, tag is lost, or timeout.
     */
    private Command rangeToTagCommand(int targetTagId, double targetDistanceMeters) {
        final double toleranceMeters = Constants.DriveConstants.RANGE_TO_TAG_TOLERANCE; // How close to the target is "close enough"?
        final double kP = Constants.DriveConstants.RANGE_TO_TAG_KP;
        final double maxVelocity = Constants.DriveConstants.MAXIMUM_TRANSLATIONAL_VELOCITY.in(Units.MetersPerSecond);
        final var request = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.Velocity);

        return Commands.run(() -> {
            System.out.println("rangeToTag: Tag ID: " + targetTagId + ", target distance: " + targetDistanceMeters + " meters");
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
                SmartDashboard.putString("Aim/RangeToTagVelocityX", "X Velocity:" + velocityX);
                SmartDashboard.putString("Aim/RangeToTagVelocityY", "Y Velocity:" + velocityY);
                SmartDashboard.putString("Aim/RangeRemainingDistance", "Remaining Distance:" + remainingDistanceToTargetMeters);
                SmartDashboard.putString("Aim/VelocityStraightToTag", "Straight Velocity:" + velocityStraightToTag);
                drivetrain.setControl(request.withVelocityX(velocityX).withVelocityY(velocityY).withRotationalRate(0.0));
            } else {     
                System.out.println("rangeToTag: Error: toTag (Translation2d) is null!");
                SmartDashboard.putString("Aim/Error", "toTag (Translation2d) is null!");
            }
        }, drivetrain)
        .until(() -> {
            double currentDistanceToTagMeters = vision.getRobotToTagTranslation(targetTagId).getNorm();
            // Only require tag to be in view when we're close; beyond ~2m cameras often can't see the tag
            final double requireTagInViewWithinMeters = Constants.DriveConstants.RANGE_TO_TAG_REQUIRE_VISION_WITHIN_METERS;
            if (currentDistanceToTagMeters >= 0 && currentDistanceToTagMeters <= requireTagInViewWithinMeters) {
                if (!vision.isTagDetected() || vision.getNearestDetectedTagId() != targetTagId) {
                    System.out.println("rangeToTag: Error: Cannot see tag (and within range of " + requireTagInViewWithinMeters + " meters)");
                    SmartDashboard.putString("Aim/Error", "Cannot see tag (and within range of " + requireTagInViewWithinMeters + " meters)");
                    // return true; // Close and we lost the tag — bail out
                }
            }
            boolean areWeThereYet = currentDistanceToTagMeters >= 0 && Math.abs(currentDistanceToTagMeters - targetDistanceMeters) <= toleranceMeters;
            if (areWeThereYet) {
                double errorMeters = currentDistanceToTagMeters - targetDistanceMeters;
                System.out.println("rangeToTag: Done! Final distance: " + String.format("%.3f", currentDistanceToTagMeters) + "m, remaining error: " + String.format("%.3f", errorMeters) + "m");
                SmartDashboard.putString("Aim/RangeToTagStatus", "Done! Final distance: " + String.format("%.3f", currentDistanceToTagMeters) + "m, remaining error: " + String.format("%.3f", errorMeters) + "m");
            } else {
                System.out.println("rangeToTag: Remaining distance: " + String.format("%.3f", currentDistanceToTagMeters - targetDistanceMeters) + "m (current " + String.format("%.3f", currentDistanceToTagMeters) + "m, target " + targetDistanceMeters + "m)");
                SmartDashboard.putString("Aim/RangeToTagStatus", "Remaining distance: " + String.format("%.3f", currentDistanceToTagMeters - targetDistanceMeters) + "m (current " + String.format("%.3f", currentDistanceToTagMeters) + "m, target " + targetDistanceMeters + "m)");
            }
            return areWeThereYet;
        })
        .withTimeout(5.0)
        .finallyDo((interrupted) -> {
                if (interrupted) {
                    System.out.println("rangeToTag: Command ended (timeout or cancelled)");
                    SmartDashboard.putString("Aim/RangeToTagError", "Timed out or cancelled");
                } else {
                    System.out.println("rangeToTag: Command ended normally");
                }
                drivetrain.setControl(request
                    .withVelocityX(0.0).withVelocityY(0.0).withRotationalRate(0.0));
            });
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