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
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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

        // Log distance traveled in X (0 when enabled) to System.out for odometry check
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
        }).ignoringDisable(true).schedule();
    }


    /**
     * Use this method to define your trigger->command mappings. 
     */
    private void configureBindings() {

        // LEFT BUMPER --> RESET LOCALIZER'S POSE
        controller.getLeftBumper().onTrue(Commands.runOnce(() -> drivetrain.getLocalizer().resetPose(new Pose2d(0,0, Rotation2d.fromRotations(0.0)))));

        // RIGHT BUMPER --> NAVIGATE FROM CURRENT POSE TO TARGET POSE (PathPlanner)
        //controller.getRightBumper().onTrue(navigateToPoseCommand(Constants.NAVIGATE_TO_POSE_TARGET));

        // D-PAD UP --> ROTATE TO FACE DETECTED APRIL TAG (fused pose)
        controller.getDPad().getUp().onTrue(Commands.runOnce(() -> {
            int id = vision.getDetectedTagId();
            if (id < 0) {
                System.out.println("rotateToTag: no AprilTag detected");
                return;
            }
            rotateToTagCommand(id).schedule();
        }));

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

            System.out.println("Attempting to rotate to tag");

            int id = vision.getNearestDetectedTagId();
            if (id < 0) {
                System.out.println("Can't rotate to AprilTag: none detected");
                return;
            }
            rotateToTagCommand(id).schedule();
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
     * Uses the fused pose estimate (combines both cameras + IMU) for accurate field-relative targeting.
     * Returns a no-op command if tagId is negative (no tag).
     */
    private Command rotateToTagCommand(int tagId) {
        
        System.out.println("Found Tag: " + tagId);

        if (tagId < 0) {
            return Commands.none();
        }

        System.out.println("Rotating to AprilTag: " + tagId);

        final int targetTagId = tagId;
        final var request = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.Velocity);
        
        // PID controller for smooth rotation alignmentf
        // Tune these values: kP controls responsiveness, kD reduces overshoot
        PIDController rotationPID = new PIDController(0.05, 0.0, 0.01);
        rotationPID.setTolerance(Math.toRadians(1.0)); // 1 degree tolerance
        rotationPID.enableContinuousInput(-Math.PI, Math.PI); // Handle wrap-around

        return Commands.run(() -> {
            double angleError = vision.getAngleToTag(targetTagId);
            double rotationalRate = rotationPID.calculate(0.0, angleError);
            double maxRotRate = Constants.DriveConstants.MAXIMUM_ROTATIONAL_VELOCITY.in(Units.RadiansPerSecond);
            rotationalRate = Math.max(-maxRotRate, Math.min(maxRotRate, rotationalRate));
            drivetrain.setControl(request
                .withVelocityX(0.0)
                .withVelocityY(0.0)
                .withRotationalRate(rotationalRate));
        }, drivetrain)
        .until(() -> {
            // Check if any tag is still detected
            if (!vision.isTagDetected()) {
                return true; // Lost tag, stop command
            }
            
            // Get tag ID from any camera
            int detectedTagId = vision.getDetectedTagId();
            if (detectedTagId < 0) {
                return true; // No tag detected
            }
            
            // Check if we're aligned (within tolerance) using fused pose
            double angleError = vision.getAngleToTag(detectedTagId);
            return Math.abs(angleError) <= Math.toRadians(1.0); // 1 degree tolerance
        })
        .finallyDo(() -> {
            rotationPID.reset();
            drivetrain.setControl(request
                .withVelocityX(0.0)
                .withVelocityY(0.0)
                .withRotationalRate(0.0));
        });
    }

}