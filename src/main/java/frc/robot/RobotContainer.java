// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
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
import frc.robot.commands.Autos;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Roller;


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
    private final Arm arm = new Arm();
    private final Roller roller = new Roller();
        
    private BreakerInputStream driverX, driverY, driverOmega;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Disable verbose logging to reduce noise
        // Flip this back on when debugging/troubleshooting
        BreakerLog.setVerboseLogging(false);

        arm.setRoller(roller);
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
                .scale(Constants.DriveConstants.MAXIMUM_ROTATIONAL_VELOCITY.in(Units.RadiansPerSecond))
                .negate();
    
        drivetrain.setDefaultCommand(drivetrain.getTeleopControlCommand(driverX, driverY, driverOmega, Constants.DriveConstants.TELEOP_CONTROL_CONFIG));

        controller.getDPad().getLeft().onTrue(rotateToTagCommand());
        controller.getDPad().getRight().onTrue(rangeToTagCommand());
        // TODO: Bind driveToTagCommand() to a controller button
        // Example: controller.getButtonX().onTrue(driveToTagCommand());

        // ---------------- ARM ----------------

        //D-PAD UP/DOWN --> Manually move arm
        controller.getDPad().getUp().whileTrue(Commands.runOnce(() -> arm.setVoltageOutput(0.1)));
        controller.getDPad().getUp().onFalse(Commands.runOnce(() -> arm.setVoltageOutput(0.0)));
        controller.getDPad().getDown().whileTrue(Commands.runOnce(() -> arm.setVoltageOutput(-0.1)));
        controller.getDPad().getDown().onFalse(Commands.runOnce(() -> arm.setVoltageOutput(0.0)));

        //RIGHT BUMPER --> Set current position as home position
        controller.getRightBumper().onTrue(Commands.runOnce(() -> arm.homePosition()));

        // ---------------- ROLLER ----------------

        //TRIGGERS --> Manually spin rollers
        controller.getLeftTrigger().whileTrue(roller.setSpeedCommand(Constants.RollerConstants.ALGAE_INTAKE_SPEED));
        controller.getLeftTrigger().onFalse(roller.setSpeedCommand(Constants.RollerConstants.IDLE_SPEED));
        controller.getRightTrigger().whileTrue(roller.setSpeedCommand(Constants.RollerConstants.ALGAE_EXTAKE_SPEED));
        controller.getRightTrigger().onFalse(roller.setSpeedCommand(Constants.RollerConstants.IDLE_SPEED));

        // ---------------- STATES ----------------

        //CORAL INTAKE
        controller.getButtonX().onTrue(
            Commands.sequence(
                arm.setStateCommand(Arm.State.UP),
                roller.setStateCommand(Roller.State.IDLE)
            )
        );

        //CORAL EXTAKE
        controller.getButtonY().onTrue(
            Commands.sequence(
                arm.setStateCommand(Arm.State.EXTAKE),
                roller.setStateCommand(Roller.State.CORAL_EXTAKE),
                Commands.waitSeconds(0.7),   // NEED TEST TODO
                roller.setStateCommand(Roller.State.IDLE)
            )
        );

        //ALGAE INTAKE
        controller.getButtonA().onTrue(
            Commands.sequence(
                arm.setStateCommand(Arm.State.DOWN),
                roller.setStateCommand(Roller.State.ALGAE_INTAKE)
            )
        );

        //ALGAE EXTAKE
        controller.getButtonB().onTrue(
            Commands.sequence(
                arm.setStateCommand(Arm.State.UP),
                roller.setStateCommand(Roller.State.ALGAE_EXTAKE)
            )
        );
    }


    public Command getAutonomousCommand() {
        return Autos.moveForward(drivetrain, roller, arm);
    }

    // AUTOALIGN TO APRIL TAG
    // TODO: Use PID controller for smooth alignment
        
    private Command rotateToTagCommand() {
        NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
        final var request = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.Velocity);

        return Commands.run(() -> {
            double xOffset = limelight.getEntry("tx").getDouble(0);
            // double rotationalRate = (-xOffset * 3.14159265358) / 180;
        
            double rotationalRate = Math.copySign(0.5, xOffset);
            drivetrain.setControl(request.withRotationalRate(rotationalRate));
        }, drivetrain).until(() -> {
            double xOffset = limelight.getEntry("tx").getDouble(0);
            return Math.abs(xOffset) <= 1.0;
        })
        .andThen(Commands.runOnce(() -> {
            drivetrain.setControl(request.withRotationalRate(0.0));
        }, drivetrain));
    }


    // AUTODRIVE TO APRIL TAG

    /**
     * Drives the robot toward the closest AprilTag using PID control for smooth alignment.
     * Controls three axes simultaneously:
     * - Rotation: Uses tx (horizontal offset) to rotate toward the tag
     * - Lateral: Uses tx to strafe left/right to get directly in front of the tag
     * - Forward/Backward: Uses tag area to control distance to the tag
     * Stops when the robot is aligned (tx ≈ 0) and at the target distance.
     */
    private Command driveToTagCommand() {
        NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
        final var request = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.Velocity);
    
        // --- TOLERANCES / TARGETS: TUNE THESE ---

        final double alignmentTolerance = 1.0; // degrees
        final double distanceTolerance = 0.1;  // meters
        final double targetTagArea = 0.8; // (larger = closer)
        final double timeoutSeconds = 5.0;

        // --- PID CONTROLLERS: TUNE THESE ---

        // Rotation PID: controls rotation based on tx (horizontal offset in radians)
        final PIDController rotationPID = new PIDController(1.15, 0.0, 0.06);
        rotationPID.setTolerance(Math.toRadians(1.0));
        
        // Lateral strafe PID: controls left/right movement based on linear lateral offset (meters)
        final PIDController lateralPID = new PIDController(0.5, 0.0, 0.05);
        lateralPID.setTolerance(0.05);
        
        // Forward PID: controls distance based on tag area
        final PIDController forwardPID = new PIDController(0.1, 0.0, 0.01);
        forwardPID.setTolerance(0.1);


        return Commands.run(() -> {

            // If we lose sight of the tag, stop moving
            double tagDetected = limelight.getEntry("tv").getDouble(0);
            if (tagDetected < 1.0) {
                drivetrain.setControl(request
                    .withVelocityX(0.0)
                    .withVelocityY(0.0)
                    .withRotationalRate(0.0));
                return;
            }
            
            // Get our current state...
            Rotation2d heading = drivetrain.getLocalizer().getPose().getRotation(); // Where are we facing, relative to the field?
            Rotation2d angleToTag = Rotation2d.fromDegrees(limelight.getEntry("tx").getDouble(0));
            double tagArea = limelight.getEntry("ta").getDouble(0); 
            double distanceToTag = 2.0 / Math.max(tagArea, 0.1); // meters, TUNE THIS

            System.out.println(
                "Heading: " + heading.getDegrees() + ", " + 
                "Angle to Tag: " + angleToTag.getDegrees() + ", " + 
                "Tag Area: " + tagArea + ", " + 
                "Distance to Tag: " + distanceToTag);

            // --- ROTATION ---

            // Use our x-offset to calculate rotation rate and use PID
            double rotationalRate = rotationPID.calculate(angleToTag.getRadians(), 0.0);


            // --- STRAFE ---

            // Estimate distance from tag area and convert angular offset to linear offset
            double lateralDistance = distanceToTag * angleToTag.getTan(); // meters
            
            // Use strafe distance as error for lateral PID (setpoint = 0 means centered)
            double lateralVelocity = lateralPID.calculate(lateralDistance, 0.0);
            
            // Use trig to convert lateral (perpendicular to robot heading) velocity to field coordinates
            Rotation2d lateralDirection = heading.plus(Rotation2d.fromDegrees(90));
            double lateralX = lateralVelocity * lateralDirection.getCos();
            double lateralY = lateralVelocity * lateralDirection.getSin();
            

            // --- FORWARD ---

            // Use tag area to calculate forward velocity using PID 
            // Negative because the camera's mounted temporarily on the back of the robot
            double forwardVelocity = -forwardPID.calculate(tagArea, targetTagArea);

            // Use trig to convert forward velocity to field coordinates
            double forwardX = forwardVelocity * heading.getCos();
            double forwardY = forwardVelocity * heading.getSin();


            // MOVE!

            double velocityX = forwardX + lateralX;
            double velocityY = forwardY + lateralY;

            System.out.println("velocityX: " + velocityX + ", velocityY: " + velocityY + ", rotationalRate: " + rotationalRate);
            
            // Move toward tag (includes both forward and lateral components), and rotate to face it
            drivetrain.setControl(request
                .withVelocityX(velocityX)
                .withVelocityY(velocityY)
                .withRotationalRate(rotationalRate));

        }, drivetrain)
        .withTimeout(timeoutSeconds)
        .until(() -> { 
            // If we lose sight of the tag, we're done
            double tv = limelight.getEntry("tv").getDouble(0);
            if (tv < 1.0) {;
                System.out.println("We lost our AprilTag: bailing out!");
                return true; 
            }
            
            // When both aligned AND at target distance, we're done
            double tx = limelight.getEntry("tx").getDouble(0);
            double ta = limelight.getEntry("ta").getDouble(0);
            boolean isAligned = Math.abs(tx) <= alignmentTolerance;
            boolean isAtTargetDistance = Math.abs(ta - targetTagArea) <= distanceTolerance;
            System.out.println(
                "angleToTag: " + tx + "(isAligned? " + isAligned + "), " +
                "tagArea: " + ta + "(isAtTargetDistance? " + isAtTargetDistance + ")");
            return (isAligned && isAtTargetDistance);
        })
        .finallyDo(() -> {
            // Reset PID controllers and stop movement
            rotationPID.reset();
            lateralPID.reset();
            forwardPID.reset();
            drivetrain.setControl(request
                .withVelocityX(0.0)
                .withVelocityY(0.0)
                .withRotationalRate(0.0));
        });
    }

    private Command rangeToTagCommand() {
        NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
        final var request = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.Velocity);
        
        // We're using a negative here because we (temporarily) have the camera on the back of the robot
        final double forwardVelocity = -0.5;

        return Commands.run(() -> {
            Rotation2d heading = drivetrain.getLocalizer().getPose().getRotation();
            
            // Convert robot-relative forward velocity to field-relative X and Y using trig
            // since SwerveRequest.FieldCentric uses field-relative coordinates
            double velocityX = forwardVelocity * heading.getCos();
            double velocityY = forwardVelocity * heading.getSin();

            drivetrain.setControl(request.withVelocityX(velocityX).withVelocityY(velocityY));

        }, drivetrain).until(() -> {
            double tagArea = limelight.getEntry("ta").getDouble(0);
            return tagArea >= 0.8;
        })
        .andThen(Commands.runOnce(() -> {
            drivetrain.setControl(request.withVelocityX(0.0).withVelocityY(0.0));
        }, drivetrain));
    }
}