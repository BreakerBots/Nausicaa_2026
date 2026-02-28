// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import static edu.wpi.first.units.Units.Meters;

import java.util.Set;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
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
    private final BreakerXboxController controller2 = new BreakerXboxController(Constants.OperatorConstants.kDriverControllerPort2);
    private final Drivetrain drivetrain = new Drivetrain();
    private final Vision vision = new Vision(drivetrain);
    private final PoseManager poseManager = new PoseManager(drivetrain, vision);
    private final TrajectoryManager trajectoryManager = new TrajectoryManager(drivetrain);
    private final Intake intake = new Intake();
    private final Climb climb = new Climb();
    private final Shooter shooter = new Shooter();
    private final Hopper hopper = new Hopper();
    
    private BreakerInputStream driverX, driverY, driverOmega;

    private boolean slowMode;
    
    /** When true, drive controls and autonomous are disabled. */
    private boolean safetyMode = false;
    
    /** PathPlanner auto chooser; populated from GUI autos when AutoBuilder is configured. */
    private final SendableChooser<Command> autoChooser;


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        // Disable verbose logging to reduce noise
        // Flip this back on when debugging/troubleshooting
        BreakerLog.setVerboseLogging(false);

        // Register named commands for PathPlanner event markers (must be before buildAutoChooser)
        NamedCommands.registerCommand("rotateToHub", Commands.defer(() -> poseManager.rotateToTagCommand(Constants.FieldConstants.getHubTagID()), Set.of(drivetrain)));
        NamedCommands.registerCommand("enterSlowMode", Commands.defer(() -> Commands.runOnce(() -> slowMode = !slowMode), Set.of(drivetrain)));
        NamedCommands.registerCommand("consolidatePose", Commands.defer(() -> Commands.runOnce(() -> drivetrain.getLocalizer().resetPose(new Pose2d(0,0, Rotation2d.fromRotations(0.0)))), Set.of(drivetrain)));
        NamedCommands.registerCommand("rangeToHub", Commands.defer(() -> poseManager.rangeToPointCommand(Constants.FieldConstants.TARGET_BLUE_HUB_CENTER, 3.0), Set.of(drivetrain)));
        NamedCommands.registerCommand("spinUp", Commands.defer(() -> shooter.setStateCommand(Shooter.State.SPINNING_UP), Set.of(shooter)));
        NamedCommands.registerCommand("shoot", Commands.defer(() -> shooter.setStateCommand(Shooter.State.SHOOTING), Set.of(shooter)));
        NamedCommands.registerCommand("stopShoot", Commands.defer(() -> shooter.setStateCommand(Shooter.State.INACTIVE), Set.of(shooter)));
        NamedCommands.registerCommand("intake", Commands.defer(() -> intake.setStateCommand(Intake.State.EXTENDED_INTAKING), Set.of(intake)));
        NamedCommands.registerCommand("stopIntake", Commands.defer(() -> intake.setStateCommand(Intake.State.EXTENDED_IDLE), Set.of(intake)));
        NamedCommands.registerCommand("halt", Commands.waitSeconds(2.0));
        NamedCommands.registerCommand("hooddown", Commands.defer(() -> shooter.hoodToRotationsCommand(Constants.ShooterConstants.POSITION_HOOD_SETPOINT_HOME), Set.of(shooter)));
        
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

        // BACK BUTTON --> SLOW MODE
        controller.getRightBumper().onTrue(Commands.runOnce(() -> slowMode = !slowMode));

        // LEFT BUMPER --> RESET LOCALIZER'S POSE
        controller.getLeftBumper().onTrue(Commands.runOnce(() -> drivetrain.getLocalizer().resetPose(new Pose2d(0,0, Rotation2d.fromRotations(0.0)))));

        // RIGHT BUMPER --> Pathfind to pose (alternative: while held; current: X+RB = on press)
        //controller.getRightBumper().whileTrue(poseManager.navigateToPoseCommand(Constants.FieldConstants.POSE_SHOOTING_BLUE_HUB_CENTER));

        //controller.getButtonX().and(controller.getRightBumper()).onTrue(poseManager.navigateToPoseCommand(Constants.FieldConstants.POSE_SHOOTING_BLUE_HUB_CENTER));



        // ---------------- SWERVE DRIVE ----------------

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
        
            // LEFT TRIGGER --> Track hub center; driver keeps X/Y, rotation follows hub
            controller.getLeftTrigger().whileTrue(poseManager.trackLeftTriggerTargetCommand(driverX, driverY));

            // Y --> Range to 2 m from hub center
            controller.getButtonY().onTrue(poseManager.rangeToPointCommand(Constants.FieldConstants.getTargetHubCenter(), 2.0));

            // Range to tag (A button)
            // controller.getButtonA().onTrue(rangeToTagCommand(Constants.FieldConstants.getTrenchTagID(), 1.0));        
        }

        // ----------------- TEST CONTROLS -------------
        
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

        // ----------------- INTAKE -------------

        // B: 
        controller.getButtonB().onTrue(Commands.runOnce(() -> {
            if (intake.state == Intake.State.EXTENDED_INTAKING) {
                CommandScheduler.getInstance().schedule(intake.setStateCommand(Intake.State.EXTENDED_IDLE));
            } else {
                // STOWED -> EXTENDED: move hood down first to clear intake path
                CommandScheduler.getInstance().schedule(
                        intake.setStateCommand(Intake.State.EXTENDED_INTAKING).alongWith(
                            shooter.hoodToRotationsCommand(Constants.ShooterConstants.POSITION_HOOD_SETPOINT_HOME)));
            }
        }, intake, shooter));
    

        // ----------------- SHOOTER + HOPPER/FEEDER -------------

        // RIGHT TRIGGER: Shoot (shooter + hopper while held; both inactive on release)
        controller.getRightTrigger().whileTrue(shootCommand());

        // D-PAD RIGHT --> Hood up (while held; stop when released)
        controller.getDPad().getRight().whileTrue(
            Commands.run(shooter::runHoodUp, shooter).finallyDo(shooter::stopHood));
        
        // D-PAD LEFT --> Hood down (while held; stop when released)
        controller.getDPad().getLeft().whileTrue(
            Commands.run(shooter::runHoodDown, shooter).finallyDo(shooter::stopHood));

        // A: Hood all the way down
        controller.getButtonA().onTrue(shooter.hoodToRotationsCommand(Constants.ShooterConstants.POSITION_HOOD_SETPOINT_HOME));

        // X: Toggle SPINNING_UP + hood setpoint 1 ↔ INACTIVE + hood down
        controller.getButtonX().onTrue(Commands.runOnce(() -> {
            if (shooter.state == Shooter.State.INACTIVE) {
                CommandScheduler.getInstance().schedule(
                    shooter.setStateCommand(Shooter.State.SPINNING_UP)
                        .andThen(shooter.hoodToRotationsCommand(Constants.ShooterConstants.POSITION_HOOD_SETPOINT_1)));
            } else {
                CommandScheduler.getInstance().schedule(
                    shooter.setStateCommand(Shooter.State.INACTIVE)
                        .andThen(shooter.hoodToRotationsCommand(Constants.ShooterConstants.POSITION_HOOD_SETPOINT_HOME)));
            }
        }, shooter));

        // //INACTIVE
        // controller.getDPad().getDown().onTrue(shooter.setStateCommand(Shooter.State.INACTIVE));

        // //SHOOTING
        // controller.getDPad().getUp().onTrue(shooter.setStateCommand(Shooter.State.SHOOTING));


        // ----------------- CLIMB -------------

        // LEFT TRIGGER --> Extend (if retracted/retracting) or Retract (if extended/extending).
        // Pressing again during an in-progress extend cancels it and retracts (and vice versa).
        // controller.getLeftTrigger().onTrue(Commands.runOnce(() -> {
        //     if (climb.isRetractedOrRetracting()) {
        //         CommandScheduler.getInstance().schedule(climb.extend());
        //     } else if (climb.isExtendedOrExtending()) {
        //         CommandScheduler.getInstance().schedule(climb.retract());
        //     }
        // }));

        // D-PAD UP --> Extend climb
        controller.getDPad().getUp().onTrue(climb.extend());

        // D-PAD DOWN --> Retract climb
        controller.getDPad().getDown().onTrue(climb.goHome());
        // controller.getDPad().getDown().and(controller.getRightBumper().negate()).onTrue(climb.retract());

        // Right Bumper --> Climb UP
        // controller.getRightBumper().onTrue(climb.ascend());


        // ---------------------------------------------
        // ---------- CONTROLLER 2 - CO-PILOT ----------
        // ---------------------------------------------

        // Setpoints for Shooting

        Trigger noDPad = new Trigger(() -> controller2.getBaseHID().getPOV() == -1);
        if (Constants.FieldConstants.isRedAlliance()) {
            controller2.getButtonX().and(controller2.getDPad().getUp()).onTrue(prepareToShootFromSetpointCommand(Constants.FieldConstants.POSE_SHOOTING_RED_L1));
            controller2.getButtonA().and(controller2.getDPad().getUp()).onTrue(prepareToShootFromSetpointCommand(Constants.FieldConstants.POSE_SHOOTING_RED_C1));
            controller2.getButtonB().and(controller2.getDPad().getUp()).onTrue(prepareToShootFromSetpointCommand(Constants.FieldConstants.POSE_SHOOTING_RED_R1));
            controller2.getButtonX().and(noDPad).onTrue(prepareToShootFromSetpointCommand(Constants.FieldConstants.POSE_SHOOTING_RED_L2));
            controller2.getButtonA().and(noDPad).onTrue(prepareToShootFromSetpointCommand(Constants.FieldConstants.POSE_SHOOTING_RED_C2));
            controller2.getButtonB().and(noDPad).onTrue(prepareToShootFromSetpointCommand(Constants.FieldConstants.POSE_SHOOTING_RED_R2));
            controller2.getButtonX().and(controller2.getDPad().getLeft()).onTrue(prepareToShootFromSetpointCommand(Constants.FieldConstants.POSE_SHOOTING_RED_L3));
            controller2.getButtonB().and(controller2.getDPad().getRight()).onTrue(prepareToShootFromSetpointCommand(Constants.FieldConstants.POSE_SHOOTING_RED_R3));
        } else {
            controller2.getButtonX().and(controller2.getDPad().getUp()).onTrue(prepareToShootFromSetpointCommand(Constants.FieldConstants.POSE_SHOOTING_BLUE_L1));
            controller2.getButtonA().and(controller2.getDPad().getUp()).onTrue(prepareToShootFromSetpointCommand(Constants.FieldConstants.POSE_SHOOTING_BLUE_C1));
            controller2.getButtonB().and(controller2.getDPad().getUp()).onTrue(prepareToShootFromSetpointCommand(Constants.FieldConstants.POSE_SHOOTING_BLUE_R1));
            controller2.getButtonX().and(noDPad).onTrue(prepareToShootFromSetpointCommand(Constants.FieldConstants.POSE_SHOOTING_BLUE_L2));
            controller2.getButtonA().and(noDPad).onTrue(prepareToShootFromSetpointCommand(Constants.FieldConstants.POSE_SHOOTING_BLUE_C2));
            controller2.getButtonB().and(noDPad).onTrue(prepareToShootFromSetpointCommand(Constants.FieldConstants.POSE_SHOOTING_BLUE_R2));
            controller2.getButtonX().and(controller2.getDPad().getLeft()).onTrue(prepareToShootFromSetpointCommand(Constants.FieldConstants.POSE_SHOOTING_BLUE_L3));
            controller2.getButtonB().and(controller2.getDPad().getRight()).onTrue(prepareToShootFromSetpointCommand(Constants.FieldConstants.POSE_SHOOTING_BLUE_R3));
        }

    }

    

    public Command prepareToShootFromSetpointCommand(Pose2d targetPose) {
        double hoodTarget = trajectoryManager.getHoodPositionForPose(targetPose);
        Command shooterPrep = shooter.setStateCommand(Shooter.State.SPINNING_UP)
                .andThen(shooter.hoodToRotationsCommand(hoodTarget));
        Command fullCommand = poseManager.navigateToPoseCommand(targetPose).alongWith(shooterPrep);
        return fullCommand.withTimeout(30.0)
                .finallyDo((interrupted) -> {
                    drivetrain.setControl(
                        new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.Velocity)
                            .withVelocityX(0).withVelocityY(0).withRotationalRate(0));
                    Command defaultDrive = drivetrain.getDefaultCommand();
                    if (defaultDrive != null) {
                        CommandScheduler.getInstance().schedule(defaultDrive);
                    }
                });
    }

   /** While held: shooter SHOOTING, hopper FEEDING; 
    * after 1s, intake jiggles LOW/HIGH every 0.5s. 
    * On release: stop feeder, shooter INACTIVE, intake EXTENDED_INTAKING. 
    */
    private Command shootCommand() {
        Command jiggleSequence = Commands.sequence(
                Commands.waitSeconds(1.0),
                Commands.sequence(
                        Commands.runOnce(() -> intake.setState(Intake.State.FEED_JIGGLE_LOW)),
                        Commands.waitSeconds(0.5),
                        Commands.runOnce(() -> intake.setState(Intake.State.FEED_JIGGLE_HIGH)),
                        Commands.waitSeconds(0.5))
                        .repeatedly()
                        .until(() -> hopper.state != Hopper.State.FEEDING));
        return Commands.parallel(
                Commands.startEnd(
                        () -> {
                            shooter.setState(Shooter.State.SHOOTING);
                            hopper.setState(Hopper.State.FEEDING);
                        },
                        () -> {
                            hopper.setState(Hopper.State.INACTIVE);
                            shooter.setState(Shooter.State.INACTIVE);
                            intake.setState(Intake.State.EXTENDED_INTAKING);
                        },
                        shooter, hopper, intake),
                jiggleSequence);
    }


    public Drivetrain getDrivetrain() {
        return drivetrain;
    }


    public void logPeriodic() {
        BreakerLog.log("SwerveDrivetrain/SafetyMode", safetyMode);
        BreakerLog.log("SwerveDrivetrain/SlowMode", slowMode);
    }


    public Command getAutonomousCommand() {
        return safetyMode ? Commands.none() : autoChooser.getSelected();
    }
    

    /** Called once when the robot enters autonomous. */
    public void autonomousInit() {
        intake.setState(Intake.State.STOWED);
        climb.setState(Climb.State.INACTIVE);
        shooter.setState(Shooter.State.INACTIVE);
        hopper.setState(Hopper.State.INACTIVE);
        // Make sure we drop the hood immediately so that the hopper extends
        CommandScheduler.getInstance().schedule(
                shooter.hoodToRotationsCommand(Constants.ShooterConstants.POSITION_HOOD_SETPOINT_HOME));
    }

    /** Called once when the robot enters teleop. */
    public void teleopInit() {
        intake.setState(Intake.State.STOWED);
        climb.setState(Climb.State.INACTIVE);
        shooter.setState(Shooter.State.INACTIVE);
        hopper.setState(Hopper.State.INACTIVE);
        CommandScheduler.getInstance().schedule(
                shooter.hoodToRotationsCommand(Constants.ShooterConstants.POSITION_HOOD_SETPOINT_HOME));
    }

}