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
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotState;
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
    private final Shooter shooter = new Shooter();
    
    private BreakerInputStream driverX, driverY, driverOmega;

    private double m_xWhenEnabled = 0;
    private boolean m_wasEnabled = false;

    /** PathPlanner auto chooser; populated from GUI autos when AutoBuilder is configured. */
    private final SendableChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Disable verbose logging to reduce noise
        // Flip this back on when debugging/troubleshooting
        BreakerLog.setVerboseLogging(true);

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

        // Log distance traveled in X (0 when enabled) to System.out for odometry check
        CommandScheduler.getInstance().schedule(
            Commands.run(() -> {
                boolean enabled = RobotState.isEnabled();
                if (enabled) {
                    if (!m_wasEnabled) {
                        m_xWhenEnabled = drivetrain.getLocalizer().getPose().getX();
                        m_wasEnabled = true;
                    }
                    double x = drivetrain.getLocalizer().getPose().getX();
                } else {
                    m_wasEnabled = false;
                }
            }).ignoringDisable(true));
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
            //int hubTagId = DriverStation.getAlliance()
            //    .map(a -> a == Alliance.Red ? Constants.FieldConstants.HUB_TAG_ID_RED : Constants.FieldConstants.HUB_TAG_ID_BLUE)
            //    .orElse(Constants.FieldConstants.HUB_TAG_ID_BLUE);
            CommandScheduler.getInstance().schedule(trackTagCommand(12));
        }));

        // ----------------- INTAKE -------------
        
        // B: EXTENDED_INTAKING ↔ EXTENDED_IDLE (toggle: if not intaking → intaking; if intaking → idle)
        controller.getButtonB().onTrue(Commands.runOnce(() -> {
            if (intake.state == Intake.State.EXTENDED_INTAKING) {
                intake.setState(Intake.State.EXTENDED_IDLE);
            } else {
                intake.setState(Intake.State.EXTENDED_INTAKING);
            }
        }, intake));


        // ----------------- HOPPER/FEEDER -------------

        // A: ?


        // ----------------- SHOOTER -------------

        // Eventually, X: SPINNING_UP + AIM (rotateToTagCommand, rangeToTagCommand)
        // Currently, X: ROTATE TO FACE NEAREST APRIL TAG
        controller.getButtonX().onTrue(Commands.runOnce(() -> {

            System.out.println("rotateToTag: Attempting to rotate to tag");
            SmartDashboard.putString("Aim/RotateToTagStatus", "Button Pressed");

            int id = vision.getNearestDetectedTagId();
            if (id < 0) {
                System.out.println("rotateToTag: Can't rotate to AprilTag, none detected");
                SmartDashboard.putString("Aim/RotateToTagStatus", "No Tag Detected");
                return;
            }
            CommandScheduler.getInstance().schedule(rotateToTagCommand(id));
        }));

        // Y: Toggle shooter state (INACTIVE ↔ SHOOTING)
        
        // //INACTIVE
        // controller.getDPad().getDown().onTrue(shooter.setStateCommand(Shooter.State.INACTIVE));

        // //SHOOTING
        // controller.getDPad().getUp().onTrue(shooter.setStateCommand(Shooter.State.SHOOTING));


        // ----------------- CLIMB -------------

        // D-PAD UP --> CLIMB UP
        // D-PAD DOWN --> CLIMB DOWN    

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
            drivetrain.setControl(request
                .withVelocityX(0.0)
                .withVelocityY(0.0)
                .withRotationalRate(0.0));
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