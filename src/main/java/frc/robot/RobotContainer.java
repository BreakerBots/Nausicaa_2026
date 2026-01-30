// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Disable verbose logging to reduce noise
        // Flip this back on when debugging/troubleshooting
        BreakerLog.setVerboseLogging(false);

        
        configureBindings();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {

        // LEFT BUMPER --> RESET LOCALIZER'S POSE
        controller.getLeftBumper().onTrue(Commands.runOnce(() -> drivetrain.getLocalizer().resetPose(new Pose2d(0,0, Rotation2d.fromRotations(0.0)))));

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
    
        // ----------------- INTAKE STATES -------------
        
        //EXTENDED INTAKING
        controller.getButtonX().onTrue(intake.setStateCommand(Intake.State.EXTENDED_INTAKING));

        //STOWED
        controller.getButtonY().onTrue(intake.setStateCommand(Intake.State.STOWED));

        //EXTENDED IDLE
        controller.getButtonA().onTrue(intake.setStateCommand(Intake.State.EXTENDED_IDLE));

        //EXTENDED EXTAKING
        controller.getButtonB().onTrue(intake.setStateCommand(Intake.State.EXTENDED_EXTAKING));

        // ----------------- SHOOTER STATES -------------

        //INACTIVE
        controller.getDPad().getDown().onTrue(shooter.setStateCommand(Shooter.State.INACTIVE));

        //SHOOTING
        controller.getDPad().getUp().onTrue(shooter.setStateCommand(Shooter.State.INACTIVE));
    }


    public Command getAutonomousCommand() {
       return null;
    }

    /**
     * Rotates the robot to face the detected AprilTag using PID control.
     * Uses the fused pose estimate (combines both cameras + IMU) for accurate field-relative targeting.
     * Tries front camera first, falls back to back camera if front doesn't see a tag.
     */
    private Command rotateToTagCommand() {
        final var request = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.Velocity);
        
        // PID controller for smooth rotation alignment
        // Tune these values: kP controls responsiveness, kD reduces overshoot
        PIDController rotationPID = new PIDController(0.05, 0.0, 0.01);
        rotationPID.setTolerance(Math.toRadians(1.0)); // 1 degree tolerance
        rotationPID.enableContinuousInput(-Math.PI, Math.PI); // Handle wrap-around

        return Commands.run(() -> {
            // Get tag ID from any camera
            int tagId = vision.getDetectedTagId();
            
            // If no tag detected, stop rotation
            if (tagId < 0) {
                drivetrain.setControl(request.withRotationalRate(0.0));
                return;
            }
            
            // Calculate angle error using fused pose and field layout (uses both cameras + IMU)
            double angleError = vision.getAngleToTag(tagId);
            
            // Use PID controller to calculate rotational rate
            // Setpoint is 0 (facing target), measurement is the angle error
            double rotationalRate = rotationPID.calculate(0.0, angleError);
            
            // Clamp rotational rate to maximum
            double maxRotRate = Constants.DriveConstants.MAXIMUM_ROTATIONAL_VELOCITY.in(Units.RadiansPerSecond);
            rotationalRate = Math.max(-maxRotRate, Math.min(maxRotRate, rotationalRate));
            
            // Apply rotation (field-centric, so no translation)
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
            int tagId = vision.getDetectedTagId();
            if (tagId < 0) {
                return true; // No tag detected
            }
            
            // Check if we're aligned (within tolerance) using fused pose
            double angleError = vision.getAngleToTag(tagId);
            return Math.abs(angleError) <= Math.toRadians(1.0); // 1 degree tolerance
        })
        .finallyDo(() -> {
            // Reset PID and stop movement
            rotationPID.reset();
            drivetrain.setControl(request
                .withVelocityX(0.0)
                .withVelocityY(0.0)
                .withRotationalRate(0.0));
        });
    }

}