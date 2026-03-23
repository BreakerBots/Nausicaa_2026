// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;


import java.util.Set;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.BreakerLib.driverstation.BreakerInputStream;
import frc.robot.BreakerLib.driverstation.BreakerInputStream2d;
import frc.robot.BreakerLib.driverstation.gamepad.controllers.BreakerXboxController;
import frc.robot.BreakerLib.util.logging.BreakerLog;
import frc.robot.BreakerLib.util.logging.BreakerLog.GitInfo;
import frc.robot.BreakerLib.util.logging.BreakerLog.Metadata;
import frc.robot.BreakerLib.util.math.functions.BreakerLinearizedConstrainedExponential;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.PoseManager;
import frc.robot.subsystems.TrajectoryManager;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems , commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    // The robot's subsystems and commands are defined here...
    private final BreakerXboxController controller = new BreakerXboxController(Constants.OperatorConstants.kDriverControllerPort);
    //private final BreakerXboxController controller2 = new BreakerXboxController(Constants.OperatorConstants.kDriverControllerPort2);
    private final Drivetrain drivetrain = new Drivetrain();
    private final Vision vision = new Vision(drivetrain);
    private final PoseManager poseManager = new PoseManager(drivetrain, vision);
    private final TrajectoryManager trajectoryManager = new TrajectoryManager(drivetrain);
    private final Intake intake = new Intake(drivetrain);
    private final Climb climb = new Climb();
    private final Shooter shooter = new Shooter(trajectoryManager);
    private final Hood hood = new Hood();
    private final Hopper hopper = new Hopper();
    
    private BreakerInputStream driverX, driverY, driverOmega;

    private boolean slowMode; // Scale down drive controls for safer driving in tight spaces
    private boolean safetyMode = false; // Drive controls and autonomous are disabled.

    /** PathPlanner auto chooser; populated from GUI autos when AutoBuilder is configured. */
    private final SendableChooser<Command> autoChooser;


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        // Register named commands for PathPlanner event markers (must be before buildAutoChooser)
        NamedCommands.registerCommand("enterSlowMode", Commands.defer(() -> Commands.runOnce(() -> slowMode = !slowMode), Set.of(drivetrain)));
        NamedCommands.registerCommand("wait", Commands.waitSeconds(1.0));
        NamedCommands.registerCommand("spinUp", Commands.defer(() -> shooter.setStateCommand(Shooter.State.SPINNING_UP), Set.of(shooter)));
        NamedCommands.registerCommand("aim", aimCommand());
        NamedCommands.registerCommand("shoot", Commands.defer(() -> shootForAutoCommand(), Set.of(shooter, hopper, intake)));
        NamedCommands.registerCommand("intake", Commands.defer(() -> intake.setStateCommand(Intake.State.EXTENDED_INTAKING), Set.of(intake)));
        NamedCommands.registerCommand("intakeExtendedIdle", Commands.defer(() -> intake.setStateCommand(Intake.State.EXTENDED_IDLE), Set.of(intake)));
        NamedCommands.registerCommand("stopIntake", Commands.defer(() -> intake.setStateCommand(Intake.State.EXTENDED_IDLE), Set.of(intake)));
        NamedCommands.registerCommand("wait4Seconds", Commands.waitSeconds(4.0));
        NamedCommands.registerCommand("hoodDown", Commands.defer(() ->
                hood.hoodToRotationsCommand(Constants.ShooterConstants.POSITION_HOOD_MIN), Set.of(hood)));
        NamedCommands.registerCommand("unclog", Commands.defer(() -> unclogCommand().withTimeout(3.0), Set.of(hopper, intake)));

        // NamedCommands.registerCommand("hooddown", Commands.none());
        // NamedCommands.registerCommand("consolidatePose", Commands.none());
        // NamedCommands.registerCommand("rotateToHub", Commands.none());
        // NamedCommands.registerCommand("rangeToHub", Commands.none());


        // Set up our auto-chooser    
        if (AutoBuilder.isConfigured()) {
            // Looks for autos in /src/main/deploy/pathplanner/autos/
            autoChooser = AutoBuilder.buildAutoChooser();
        } else {
            autoChooser = new SendableChooser<>();
            autoChooser.setDefaultOption("Do Nothing", Commands.none());
        }
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // Set up our logs
        configureLogging();
        // Disable verbose logging to reduce noise
        // Flip this back on when debugging/troubleshooting
        BreakerLog.setVerboseLogging(false);

        // Bind our controller buttons
        configureBindings();
    }

    private void configureLogging() {

        // Phoenix 6 normally logs CAN bus data
        //SignalLogger.enableAutoLogging(false);
        
        // Set up DogLog options for logging
        // DogLogOptions normalOptions = new DogLogOptions().withLogExtras(true).withCaptureDs(true).withNtPublish(true);
        // DogLogOptions competitionOptions = new DogLogOptions().withLogExtras(true).withCaptureDs(true).withNtPublish(false);
        // BreakerLog.setOptions(normalOptions);
        
        // When logExtras is true...
        // Log data from PDH (current draw, voltage, temperature, etc.)
        //BreakerLog.setPdh(new PowerDistribution(MiscConstants.PDH_ID, ModuleType.kRev));
       
        // Log data from CAN bus
        //BreakerLog.addCANBus(Constants.GeneralConstants.DRIVE_CANIVORE_BUS);
        
        // Adds context to logs so we know what robot, git commit,  etc.
        GitInfo gitInfo = new GitInfo(BuildConstants.MAVEN_NAME, BuildConstants.GIT_REVISION, BuildConstants.GIT_SHA, BuildConstants.GIT_DATE, BuildConstants.GIT_BRANCH, BuildConstants.BUILD_DATE, BuildConstants.DIRTY);
        BreakerLog.logMetadata(new Metadata("Nausicaa", 2026, "Isaac Lynch, Max Xu, Matthew Pedersen, Paul Brockmeyer", gitInfo));
      }


    /**
     * Use this method to define your trigger->command mappings.
     *
     * Tips:
     * - Simplest: 
     *     Default to using onTrue(command), when there's no logic and you have a command.
     *     Action only while held? Use whileTrue(command).
     * - If you don't have a command and need a one-shot action (setState, toggle, zero encoder)? Use runOnce(() -> action(), subsystem).
     * - If you need if/else to determine which command to run? Use runOnce with CommandScheduler.getInstance().schedule() inside.
     */
    private void configureBindings() {

        // ---------------------------------------------
        // ---------- CONTROLLER 1 - DRIVER ----------
        // ---------------------------------------------

        // ---------------- THUMBSTICKS - DRIVE ----------------

        // Don't bind these is it's not safe to drive (ie. when the robot is on a table)
        if (!safetyMode) {
        
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
        
            // LEFT/RIGHT THUMBSTICK HOLD --> SLOW MODE
            // controller.getLeftThumbstick().getJoystickButton().whileTrue(
            //     Commands.startEnd(() -> slowMode = true, () -> slowMode = false));
            // controller.getRightThumbstick().getJoystickButton().whileTrue(
            //     Commands.startEnd(() -> slowMode = true, () -> slowMode = false));
        }


        // ----------------- BUMPERS -------------

        // LEFT BUMPER --> RESET LOCALIZER'S POSE
        controller.getLeftBumper().onTrue(Commands.runOnce(() -> drivetrain.getLocalizer().resetPose(new Pose2d(0,0, Rotation2d.fromRotations(0.0)))));

        // RIGHT BUMPER --> Open?
        controller.getRightBumper().onTrue(Commands.runOnce(() -> slowMode = !slowMode));


        // ----------------- TRIGGERS -------------

        // LEFT TRIGGER --> Intake (while held; stop when released)
        controller.getLeftTrigger().whileTrue(Commands.run(() -> intake.setState(Intake.State.EXTENDED_INTAKING), intake)
            .finallyDo(() -> intake.setState(Intake.State.EXTENDED_IDLE)));
            
        // RIGHT TRIGGER --> Continuously aim and shoot while held; both stop when released
        //controller.getRightTrigger().whileTrue(aimThenShootCommand());
        controller.getRightTrigger().whileTrue(aimAndShootContinuouslyCommand());
    

        // ----------------- BUTTONS -------------

        // A --> Just Aim
        controller.getButtonA().onTrue(aimCommand());
        
        // B --> Just Shoot
        controller.getButtonB().onTrue(shootForTeleopCommand(true));

        // X --> Hood to Latch Position
        //controller.getButtonX().onTrue(shooter.hoodToRotationsCommand(Constants.ShooterConstants.POSITION_HOOD_LATCH));
        controller.getButtonX().onTrue(intake.setStateCommand(Intake.State.STOWED));

        // Y --> Unclog: run feeder, indexer, and intake in reverse at 20% speed (while held)
        controller.getButtonY().whileTrue(unclogCommand());


        // -- FOR RECORDING SHOOTER DATA --

        // Y --> Hood to setpoint
        //controller.getButtonY().onTrue(shooter.hoodToRotationsCommand(Constants.ShooterConstants.POSITION_HOOD_LATCH));

        // A --> Hood all the way down
        //controller.getButtonA().onTrue(shooter.hoodToRotationsCommand(Constants.ShooterConstants.POSITION_HOOD_MIN));
        

        // ----------------- DPAD-------------

        // D-PAD RIGHT --> Hood up (while held; stop when released)
        controller.getDPad().getRight().whileTrue(
            Commands.run(hood::runHoodUp, hood).finallyDo(hood::stopHood));
        
        // D-PAD LEFT --> Hood down (while held; stop when released)
        controller.getDPad().getLeft().whileTrue(
            Commands.run(hood::runHoodDown, hood).finallyDo(hood::stopHood));


        // D-PAD UP --> Run climb up (while held)
        controller.getDPad().getUp().whileTrue(climb.runUp());

        // D-PAD DOWN --> Run climb down (while held)
        controller.getDPad().getDown().whileTrue(climb.runDown());



        // ---------------------------------------------
        // ---------- OLD ----------
        // ---------------------------------------------

        // X: Toggle SPINNING_UP + hood setpoint 1 ↔ INACTIVE + hood down
        // controller.getButtonX().onTrue(Commands.runOnce(() -> {
        //     if (shooter.state == Shooter.State.INACTIVE) {
        //         CommandScheduler.getInstance().schedule(
        //             shooter.setStateCommand(Shooter.State.SPINNING_UP));
        //                 //.andThen(shooter.hoodToRotationsCommand(Constants.ShooterConstants.POSITION_HOOD_SETPOINT_1)));
        //     } else {
        //         CommandScheduler.getInstance().schedule(
        //             shooter.setStateCommand(Shooter.State.INACTIVE));
        //                 //.andThen(shooter.hoodToRotationsCommand(Constants.ShooterConstants.POSITION_HOOD_SETPOINT_HOME)));
        //     }
        // }, shooter));

        // B: ROLLER
        // controller.getButtonB().onTrue(Commands.runOnce(() -> {
        //     if (intake.state == Intake.State.EXTENDED_INTAKING) {
        //         CommandScheduler.getInstance().schedule(intake.setStateCommand(Intake.State.EXTENDED_IDLE));
        //     } else {
        //         CommandScheduler.getInstance().schedule(intake.setStateCommand(Intake.State.STOWED));
        //     }
        // }, intake));

        // B: STOWED → EXTENDED_IDLE → EXTENDED_INTAKING → EXTENDED_IDLE → ... (saved for later)
        // controller.getButtonB().onTrue(Commands.runOnce(() -> {
        //     switch (intake.state) {
        //         case STOWED -> CommandScheduler.getInstance().schedule(intake.setStateCommand(Intake.State.EXTENDED_IDLE));
        //         case EXTENDED_IDLE -> CommandScheduler.getInstance().schedule(intake.setStateCommand(Intake.State.EXTENDED_INTAKING));
        //         case EXTENDED_INTAKING -> CommandScheduler.getInstance().schedule(intake.setStateCommand(Intake.State.EXTENDED_IDLE));
        //         default -> CommandScheduler.getInstance().schedule(intake.setStateCommand(Intake.State.EXTENDED_IDLE));
        //     }
        // }, intake));

        // X/B --> Short-range navigate-to-trench, with NZ + max-distance protection handled in PoseManager.
        // if (!safetyMode) {
        //     controller.getButtonX().onTrue(
        //         poseManager.navigateToTrench(Constants.FieldConstants.getTrenchLeftExitPose()));
        //     controller.getButtonB().onTrue(
        //         poseManager.navigateToTrench(Constants.FieldConstants.getTrenchRightExitPose()));
        // }

        // LEFT TRIGGER --> Track hub center; driver keeps X/Y, rotation follows hub; hood tracks distance
        // DoubleSupplier targetDistance = () -> drivetrain.getRobotToPointTranslation(
        //         Constants.FieldConstants.getTargetForPose(drivetrain.getLocalizer().getPose())).getNorm();
        // controller.getLeftTrigger().whileTrue(
        //     //drivetrain.getLocalizer().resetPose(new Pose2d(0,0, Rotation2d.fromRotations(0.0))))
        //         //  .andThen(poseManager.trackLeftTriggerTargetCommand(driverX, driverY)
        //     poseManager.trackTargetCommand(driverX, driverY)
        //         .alongWith(shooter.positionHoodContinuouslyCommand(targetDistance)));        
    }


   /** Unclog: run feeder and indexer in reverse at ~20% speed; intake extaking. While held. */
    private Command unclogCommand() {
        return Commands.startEnd(
                () -> {
                    hopper.setState(Hopper.State.UNCLOG);
                    intake.setState(Intake.State.EXTENDED_EXTAKING);
                },
                () -> {
                    hopper.setState(Hopper.State.INACTIVE);
                    intake.setState(Intake.State.EXTENDED_IDLE);
                },
                hopper, intake);
    }

    /**
     * Aim (rotate + hood), then shoot. Runs aim first, then shoot with wheels locked for stability.
     */
    private Command aimThenShootCommand() {
        return aimCommand().andThen(drivetrain.lockWheelsCommand().raceWith(shootForTeleopCommand()));
    }

    /**
     * Aim and shoot simultaneously. Runs aimCommand and shootForTeleopCommand in parallel.
     * No wheel lock (aim needs to rotate).
     */
    // private Command aimAndShootCommand() {
    //     return Commands.parallel(aimCommand(), shootForTeleopCommand());
    // }

    /**
     * Aim once (rotate + hood), then shoot while held. Aim runs to completion; shoot runs until trigger released.
     * ignoreAimError=true so feeder runs regardless of angle (vision may not be aligned).
     */
    private Command aimAndShootContinuouslyCommand() {
        return Commands.parallel(aimContinuouslyCommand(false), shootForTeleopCommand(true));
    }


    
    /** Auto shoot: feed phase runs for 6 seconds. Locks wheels for stability during shoot. */
    private Command shootForAutoCommand() {
        //return drivetrain.lockWheelsCommand().raceWith(
        return shootSequenceCommand(6.0);
            //);
    }

    /** Teleop shoot: feed phase runs until trigger released. */
    private Command shootForTeleopCommand() {
        return shootForTeleopCommand(false);
    }

    /**
     * Teleop shoot with optional angle bypass. When ignoreAngleError is true, feeds regardless of heading to target.
     */
    private Command shootForTeleopCommand(boolean ignoreAimError) {
        return shootSequenceCommand(null, ignoreAimError);
    }


    /**
     * Don't use this command directly. Use either shootForAutoCommand() or shootForTeleopCommand() instead.
     */
    private Command shootSequenceCommand(Double feedTimeoutSeconds) {
        return shootSequenceCommand(feedTimeoutSeconds, false);
    }

    private Command shootSequenceCommand(Double feedTimeoutSeconds, boolean ignoreAimError) {
        Command jiggleSequence = Commands.sequence(
                Commands.waitSeconds(1.0),
                Commands.sequence(
                        Commands.runOnce(() -> intake.setState(Intake.State.FEED_JIGGLE_LOW)),
                        Commands.waitSeconds(0.3),
                        Commands.runOnce(() -> intake.setState(Intake.State.FEED_JIGGLE_HIGH)),
                        Commands.waitSeconds(0.3))
                        .repeatedly());
        // No .until() — keeps jiggling when feedControl pauses hopper for angle error; ends when trigger released.

        Command feedControl = Commands.run(() -> {
                    shooter.setState(Shooter.State.SHOOTING);
                    if (ignoreAimError) {
                        hopper.setState(Hopper.State.FEEDING);
                    } else {
                        Translation2d target = Constants.FieldConstants.getTargetForPose(drivetrain.getLocalizer().getPose());
                        double angleErrorRad = Math.abs(vision.getAngleToTarget(target));
                        if (angleErrorRad <= Constants.ShooterConstants.FEED_PAUSE_ANGLE_THRESHOLD_RAD) {
                            hopper.setState(Hopper.State.FEEDING);
                        } else {
                            hopper.setState(Hopper.State.INACTIVE);
                            //intake.setState(Intake.State.EXTENDED_IDLE);
                        }
                    }
                }, shooter, hopper); //intake

        Command feedPhase = Commands.parallel(feedControl, jiggleSequence);

        Command feedPhaseWithDuration = feedTimeoutSeconds != null
                ? feedPhase.withTimeout(feedTimeoutSeconds)
                : feedPhase;

        // Spin up, but don't wait forever for flywheels to reach target speed.
        // If they aren't at speed within the timeout, we proceed anyway.
        Command shootSequence = Commands.sequence(
                        Commands.runOnce(() -> shooter.setState(Shooter.State.SPINNING_UP), shooter),
                        Commands.waitUntil(shooter::isAtTargetSpeed).withTimeout(0.5),
                        feedPhaseWithDuration)
                .finallyDo((interrupted) -> {
                    hopper.setState(Hopper.State.INACTIVE);
                    shooter.setState(Shooter.State.INACTIVE);
                    intake.setState(Intake.State.EXTENDED_IDLE);
                    CommandScheduler.getInstance().schedule(
                            hood.hoodToRotationsCommand(Constants.ShooterConstants.POSITION_HOOD_MIN));
                });
        return shootSequence;
    }


    /**
     * One-time aim: rotate to face target and position hood for distance. Both run in parallel until complete.
     */
    private Command aimCommand() {
        double[] startTime = new double[1];
        return Commands.defer(() -> {
            Pose2d pose = drivetrain.getLocalizer().getPose();
            Translation2d target = Constants.FieldConstants.getTargetForPose(pose);
            double distance = drivetrain.getRobotToPointTranslation(target).getNorm();
            double hoodTarget = TrajectoryManager.getHoodPositionForDistance(distance);

            double[] rotateStart = new double[1];
            Command rotateCmd = poseManager.rotateToPointCommand(target)
                .beforeStarting(() -> rotateStart[0] = Timer.getFPGATimestamp())
                .finallyDo((interrupted) -> {
                    double elapsed = Timer.getFPGATimestamp() - rotateStart[0];
                    BreakerLog.log("Aim/RotateToPoint/ElapsedSeconds", elapsed);
                    BreakerLog.log("Aim/RotateToPoint/Interrupted", interrupted);
                });

            double[] hoodStart = new double[1];
            Command hoodCmd = hood.hoodToRotationsCommand(hoodTarget)
                    .beforeStarting(() -> hoodStart[0] = Timer.getFPGATimestamp())
                    .finallyDo((interrupted) -> {
                        double elapsed = Timer.getFPGATimestamp() - hoodStart[0];
                        BreakerLog.log("Aim/HoodToRotations/ElapsedSeconds", elapsed);
                        BreakerLog.log("Aim/HoodToRotations/Interrupted", interrupted);
                    });

            return Commands.parallel(rotateCmd, hoodCmd);
        }, Set.of(drivetrain, hood))
            .withTimeout(1.0)
            .beforeStarting(() -> startTime[0] = Timer.getFPGATimestamp())
            .finallyDo((interrupted) -> {
                double elapsed = Timer.getFPGATimestamp() - startTime[0];
                BreakerLog.log("Aim/ElapsedSeconds", elapsed);
                BreakerLog.log("Aim/Interrupted", interrupted);
            });
    }

    /**
     * Continuously aim: track target heading (rotation). Hood positioned once at start (or continuously if adjustHoodContinuously).
     * Runs until interrupted.
     * @param adjustHoodContinuously when true, hood tracks distance continuously; when false (default), hood set once at start
     */
    private Command aimContinuouslyCommand(boolean positionHoodContinuously) {
        Command trackCmd = poseManager.rotateToPointContinuouslyCommand(
                () -> Constants.FieldConstants.getTargetForPose(drivetrain.getLocalizer().getPose()),
                driverX,
                driverY);
        Command hoodCmd = positionHoodContinuously
                ? positionHoodContinuouslyCommand()
                : Commands.defer(() -> {
                    double distance = drivetrain.getRobotToPointTranslation(
                            Constants.FieldConstants.getTargetForPose(drivetrain.getLocalizer().getPose())).getNorm();
                    double hoodTarget = TrajectoryManager.getHoodPositionForDistance(distance);
                    return hood.hoodToRotationsCommand(hoodTarget);
                }, Set.of(hood));
        return Commands.parallel(trackCmd, hoodCmd);
    }


    /**
     * Positions hood based on distance to target for current pose. Distance is re-evaluated each cycle.
     * Uses getTargetForPose(pose) to determine target, then delegates to shooter.
     */
    public Command positionHoodContinuouslyCommand() {
        return hood.positionHoodContinuouslyCommand(() ->
                drivetrain.getRobotToPointTranslation(
                        Constants.FieldConstants.getTargetForPose(drivetrain.getLocalizer().getPose())).getNorm());
    }


    public Drivetrain getDrivetrain() {
        return drivetrain;
    }


    public void logPeriodic() {
        BreakerLog.log("SwerveDrivetrain/SafetyMode", safetyMode, true);
        BreakerLog.log("SwerveDrivetrain/SlowMode", slowMode, true);
        
        BreakerLog.log("DistanceToTarget", drivetrain.getRobotToPointTranslation(
            Constants.FieldConstants.getTargetForPose(drivetrain.getLocalizer().getPose())).getNorm(), true);
        BreakerLog.log("DistanceFromRobotFrontToTarget", drivetrain.getRobotToPointTranslation(
                Constants.FieldConstants.getTargetForPose(drivetrain.getLocalizer().getPose())).getNorm() - 0.39878, true);

        // Aim telemetry: distance, hood target/actual, flywheel target, heading error
        Translation2d aimTarget = Constants.FieldConstants.getTargetForPose(drivetrain.getLocalizer().getPose());
        double aimDistanceM = drivetrain.getRobotToPointTranslation(aimTarget).getNorm();
        BreakerLog.log("Aim/DistanceToTargetM", aimDistanceM, true);
        BreakerLog.log("Aim/HoodTargetRot", TrajectoryManager.getHoodPositionForDistance(aimDistanceM), true);
        BreakerLog.log("Aim/HoodPositionRot", hood.getHoodEncoderRotations(), true);
        BreakerLog.log("Aim/FlywheelTargetRps", trajectoryManager.getFlywheelSpeedForDistance(), true);
        BreakerLog.log("Aim/HeadingErrorDeg", Math.toDegrees(vision.getAngleToTarget(aimTarget)), true);

        MatchTimer.update();
    }


    public Command getAutonomousCommand() {
        return safetyMode ? Commands.none() : autoChooser.getSelected();
    }
    

    /** Called once when the robot enters autonomous. */
    public void autonomousInit() {

        intake.setState(Intake.State.STOWED);
        //climb.setState(Climb.State.INACTIVE);
        shooter.setState(Shooter.State.INACTIVE);
        hopper.setState(Hopper.State.INACTIVE);
        CommandScheduler.getInstance().schedule(
                hood.hoodToRotationsCommand(Constants.ShooterConstants.POSITION_HOOD_MIN));
    }                           

    /** Called once when the robot enters teleop. */
    public void teleopInit() {
        
        //intake.setState(Intake.State.STOWED);
        if (intake.state != Intake.State.STOWED) {
            intake.setState(Intake.State.EXTENDED_IDLE);
        } else {
            intake.setState(Intake.State.STOWED);
        }    
        climb.setState(Climb.State.INACTIVE);
         shooter.setState(Shooter.State.INACTIVE);
        hopper.setState(Hopper.State.INACTIVE);
        CommandScheduler.getInstance().schedule(hood.hoodToRotationsCommand(Constants.ShooterConstants.POSITION_HOOD_MIN));
    }
}