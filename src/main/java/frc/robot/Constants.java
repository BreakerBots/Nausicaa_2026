// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import java.util.Optional;

import frc.robot.BreakerLib.swerve.BreakerSwerveDrivetrain.BreakerSwerveDrivetrainConstants;
import frc.robot.BreakerLib.swerve.BreakerSwerveDrivetrain.BreakerSwerveDrivetrainConstants.ChoreoConfig;
import frc.robot.BreakerLib.swerve.BreakerSwerveDrivetrain.BreakerSwerveDrivetrainConstants.PathplannerConfig;
import frc.robot.BreakerLib.swerve.BreakerSwerveTeleopControl.HeadingCompensationConfig;
import frc.robot.BreakerLib.swerve.BreakerSwerveTeleopControl.TeleopControlConfig;
import frc.robot.BreakerLib.util.logging.BreakerLog.GitInfo;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. 
 * This class should not be used for any other purpose. 
 * All constants should be declared globally (i.e. public static). 
 * Do not put anything functional in this class.
 *
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    // ---------------- GENERAL ----------------

    public static class GeneralConstants {
        public static final CANBus DRIVE_CANIVORE_BUS = new CANBus("rio");
        public static final CANBus SUPERSTRUCTURE_CANIVORE_BUS = new CANBus("superstructure");
        public static final GitInfo GIT_INFO = new GitInfo(BuildConstants.MAVEN_NAME, BuildConstants.GIT_REVISION,
            BuildConstants.GIT_SHA, BuildConstants.GIT_DATE, BuildConstants.GIT_BRANCH, BuildConstants.BUILD_DATE,
            BuildConstants.DIRTY);
    }

    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
        public static final double TRANSLATIONAL_DEADBAND = 0.1;
        public static final double ROTATIONAL_DEADBAND = 0.1;
    }

    // ---------------- VISION ----------------

    public static class VisionConstants {

        public static final AprilTagFieldLayout kAprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

        // Vision system selection: true = MegaTag2, false = MegaTag1
        // MegaTag2: Uses IMU fusion for improved accuracy, requires SetRobotOrientation() calls
        // MegaTag1: Original AprilTag localization, no IMU fusion required
        public static final boolean USE_MEGATAG2 = false;

        // Limelight 4 camera names (configured in Limelight UI)
        public static final String FRONT_CAMERA = "limelight-fr";
        public static final String BACK_CAMERA = "limelight-bl";
        
        // Camera pose relative to robot center (meters, degrees)
        // Format: [forward, side, up, roll, pitch, yaw]
        // TODO: Measure and configure actual camera positions
        //public static final double[] FRONT_CAMERA_POSE = {0.244983, 0.3155442, 0.2014728, 0.0, 23.0, 34.0};
        //public static final double[] BACK_CAMERA_POSE = {-0.244983, -0.3155442, 0.2014728, 0.0, 23.0, -146.0};
        public static final double[] FRONT_CAMERA_POSE = {0.32385, 0.24765, 0.2413, 0.0, 23.0, 34.0};
        public static final double[] BACK_CAMERA_POSE = {-0.32385, -0.24765, 0.2413, 0.0, 23.0, -146.0};
        
        // Minimum number of tags required to trust a vision measurement
        public static final int MIN_TAG_COUNT = 1;
        
        // Maximum pose difference from current estimate to accept vision measurement (meters)
        public static final double MAX_POSE_DIFFERENCE = 6.0;

        // Vision measurement standard deviations (x, y, theta)
        // Increase these values to trust vision measurements less
        // Units: meters for x/y, radians for theta
        // Theta: 9999999 to fully trust IMU for rotation (vision won't correct heading)
        public static final Matrix<N3, N1> VISION_STD_DEVS = 
            VecBuilder.fill(0.05, 0.05, 0.05);        
        
        // Dynamic standard deviation scaling factors (tag count + proximity)
        // trustScore = tagCount * TAG_COUNT_SCALE_FACTOR + PROXIMITY_SCALE_FACTOR / (1 + avgTagDist)
        // stdDev = baseStdDev / (1 + trustScore)  --> higher trustScore = more trust = lower std dev
        // More tags and closer tags both increase trust. A camera with 1 close tag can beat 2 far tags.
        public static final double TAG_COUNT_SCALE_FACTOR = 0.5;
        /** How much proximity boosts trust. avgTagDist is in meters. At 1m: 1/(1+1)=0.5 boost. At 4m: 1/5=0.2 boost. */
        public static final double PROXIMITY_SCALE_FACTOR = 2.0;
    }


    // --------------- FIELD --------------

    public static class FieldConstants {

        /** Target pose for tele-op "navigate to" command. PathPlanner pathfinds from current position to this pose. Tune x, y, rotation as needed. */
        public static final Pose2d POSE_BLUE_HUB_CENTER = new Pose2d(3.0, 4.0, Rotation2d.fromDegrees(0));
        public static final Pose2d POSE_BLUE_CLIMB_READY = new Pose2d(1.05, 4.75, Rotation2d.fromDegrees(180));

        //public static final int HUB_TAG_ID_RED = 12;
        //public static final int HUB_TAG_ID_BLUE = 12;
        public static final int HUB_TAG_ID_RED = 10;
        public static final int HUB_TAG_ID_BLUE = 26;

        public static final int TRENCH_TAG_ID_RED = 12;
        public static final int TRENCH_TAG_ID_BLUE = 28;

        /** Returns the hub AprilTag ID for the current alliance (red or blue). Defaults to blue when alliance is not assigned. */
        public static int getHubTagID() {
            if (getAlliance().isPresent() && getAlliance().get() == Alliance.Red) {
                return HUB_TAG_ID_RED;
            } else {
                return HUB_TAG_ID_BLUE;
            }
        }

        public static int getTrenchTagID() {
            if (getAlliance().isPresent() && getAlliance().get() == Alliance.Red) {
                return TRENCH_TAG_ID_RED;
            } else {
                return TRENCH_TAG_ID_BLUE;
            }
        }

        public static Optional<Alliance> getAlliance() {
            Optional<Alliance> alliance = DriverStation.getAlliance();
            return alliance;
        }
    }


    // --------------- INTAKE --------------

    public static class IntakeConstants {
        public static final int PIVOT_MOTOR_ID = 20;
        public static final int ROLLER_MOTOR_ID = 21;
        public static final int PIVOT_ENCODER_ID = 25;

        /** Pivot angles (rotations) – placeholders until tuned. */
        public static final Rotation2d POSITION_STOWED = Rotation2d.fromRotations(0.0);
        public static final Rotation2d POSITION_EXTENDED = Rotation2d.fromRotations(0.18);
        public static final Rotation2d POSITION_JIGGLE_HIGH = Rotation2d.fromRotations(0.0);
        public static final Rotation2d POSITION_JIGGLE_LOW = Rotation2d.fromRotations(0.0);

        /** Motion Magic (rotations/s, rotations/s², rotations/s³). */
        public static final double PIVOT_MM_CRUISE_VELOCITY = 1.0;
        public static final double PIVOT_MM_ACCELERATION = 2.0;
        public static final double PIVOT_MM_JERK = 0.5;

        /** Feedforward (Slot0). */
        public static final double PIVOT_kS = 0.08;
        public static final double PIVOT_kG = 0.00;
        public static final double PIVOT_kV = 0.12;
        public static final double PIVOT_kA = 0.01;

        /** PID (Slot0). */
        public static final double PIVOT_kP = 0.4;
        public static final double PIVOT_kI = 0.00;
        public static final double PIVOT_kD = 0.08;

        public static final double SPEED_IDLE = 0;
        public static final double SPEED_EXTAKE = 0.5;
        public static final double SPEED_INTAKE = -0.5;
    }

    // --------------- SHOOTER --------------

    public static class ShooterConstants {
        public static final int SHOOTER_FLYWHEEL_1_MOTOR_ID = 30;
        public static final int SHOOTER_FLYWHEEL_2_MOTOR_ID = 31;
        public static final int SHOOTER_FLYWHEEL_3_MOTOR_ID = 32;
        public static final int HOOD_MOTOR_ID = 33;
        public static final int HOOD_ENCODER_ID = 35;
        public static final double SPEED_IDLE = 0;
        public static final double SPEED_FLYWHEEL_ACTIVE = 1.00; // need to tune

        /** Hood: external encoder; command takes target rotations. */
        public static final double SPEED_HOOD_UP = 0.08; // need to tune
        public static final double SPEED_HOOD_DOWN = -0.08; // need to tune

        
    }

    // --------------- HOPPER --------------

    public static class HopperConstants {
        public static final int HOPPER_MOTOR_ID = 40;
        public static final int FEEDER_MOTOR_ID = 41;
        public static final double SPEED_INACTIVE = 0;
        public static final double SPEED_INDEXING = 0.5; // need to tune
        public static final double SPEED_FEEDING = 0.6; // need to tune
    }

    // --------------- CLIMB --------------
    // Climb uses setpoint-based control with external encoder (or motor encoder as fallback).

    public static class ClimbConstants {
        public static final int CLIMB_MOTOR_ID = 50;
        public static final int CLIMB_ENCODER_ID = 55;

        /** If true, use motor's integrated encoder instead of external CANcoder. */
        /** Setpoints: encoder positions (number of rotations) for UP and DOWN positions. */
        public static final double SETPOINT_UP = -1.788125;
        public static final double SETPOINT_CLIMBED = -1.5;
        public static final double SETPOINT_DOWN = 0.1;

        /**
         * DOWN:
         * Magnetic offset: -0.3642578125
         * Absolute Position: 0.000244
         * Absolute Position No Offset: 0.294434
         * 
         * UP:
         * same magnetic offset
         * Absolute Position: 0.297363
         * Absolute Position No Offset: -1.408691
         * 
         */
        
        /** Tolerance: how close is close enough (rotations). */
        public static final double SETPOINT_TOLERANCE = 0.1;

        /** Faster speeds for extending/retracting. */
        public static final double SPEED_EXTENDING = 0.2; // need to tune
        public static final double SPEED_RETRACTING = -0.2; // need to tune
        
        /** Slower speeds for ascending/descending (0.2 = 20% motor power) */
        public static final double SPEED_ASCENDING = -0.2;
        public static final double SPEED_DESCENDING = 0.2;

        /** PID gains for setpoint control. */
        public static final double PID_kP = 4.0;
        public static final double PID_kI = 0.0;
        public static final double PID_kD = 0.0;
    }

    // ---------------- SWERVE DRIVE ----------------

    public static class DriveConstants {
        
        /** ROBOT-LEVEL MAXIMUM SPEEDS - How fast can the robot drive and rotate (currently reduced for testing)
         * Translational = forward/backward and left/right movement (X and Y on the field)
         * Rotational = spinning in place (turning) */
        public static final LinearVelocity MAXIMUM_TRANSLATIONAL_VELOCITY = Units.MetersPerSecond.of(1);
        public static final LinearVelocity ALIGN_MODE_MAXIMUM_TRANSLATIONAL_VELOCITY = Units.MetersPerSecond.of(0.3);
        public static final AngularVelocity MAXIMUM_ROTATIONAL_VELOCITY = Units.RadiansPerSecond.of(2);
        public static final AngularVelocity ALIGN_MODE_MAXIMUM_ROTATIONAL_VELOCITY = Units.RadiansPerSecond.of(0.6);
        //public static final LinearVelocity MAXIMUM_TRANSLATIONAL_VELOCITY = Units.MetersPerSecond.of(4.5);
        //public static final AngularVelocity MAXIMUM_ROTATIONAL_VELOCITY = Units.RadiansPerSecond.of(9.5);

        /** HEADING COMPENSATION = automatically maintains robot heading when driver isn't rotating 
         *  Adjust PID values if robot drifts when driver isn't rotating, or fights driver input too much */
        public static final HeadingCompensationConfig HEADING_COMPENSATION_CONFIG = new HeadingCompensationConfig(
            Units.MetersPerSecond.of(0.05),
            Units.RadiansPerSecond.of(0.001),
            Units.Seconds.of(0.2),
            new PIDConstants(1.5, 0, 0));// 1.5
        
        /** MODULE ROTATION SPEED (at the module level, Azimuth) - How fast can each swerve module turn? */
        //public static final AngularVelocity MAXIMUM_MODULE_AZIMUTH_SPEED = Units.DegreesPerSecond.of(720);
        //public static final SetpointGenerationConfig SETPOINT_GENERATION_CONFIG = new SetpointGenerationConfig(MAXIMUM_MODULE_AZIMUTH_SPEED);

        public static final TeleopControlConfig TELEOP_CONTROL_CONFIG = new TeleopControlConfig();
            // .withHeadingCompensation(HEADING_COMPENSATION_CONFIG);
            // .withSetpointGeneration(SETPOINT_GENERATION_CONFIG);

        public static final double RANGE_TO_TAG_TOLERANCE = 0.1; // Close enough to target distance, meters
        public static final double RANGE_TO_TAG_MAX_DISTANCE = 5.0; // Maximum distance to target, meters
        /** Proportional gain for range-to-tag: position error (m) → velocity (m/s). Matches PathPlanner translation for consistency. */
        public static final double RANGE_TO_TAG_KP = 4.0;
        /** Only require the tag to be in camera view when within this distance (m). Beyond this, use fused pose + field layout; don't bail on "tag lost" since cameras often can't see tags at range. */
        public static final double RANGE_TO_TAG_REQUIRE_VISION_WITHIN_METERS = 2;

        // Motor control gains: PID and feedforward values for steer and drive motors
        // Steer motor = rotates the swerve module (azimuth/steering)
        // Drive motor = spins the wheel (forward/backward movement)
        // PID gains (KP, KI, KD) = how aggressively the motor corrects position errors
        // Feedforward gains (KS, KV, KA) = predict voltage needed for desired motion (reduces lag)
        // TUNING: Adjust if modules overshoot targets, oscillate, or respond too slowly
        private static final Slot0Configs steerGains = new Slot0Configs()
            .withKP(100).withKI(0).withKD(0.2)
            .withKS(0).withKV(1.5).withKA(0);
        private static final Slot0Configs driveGains = new Slot0Configs()
            .withKP(0.01).withKI(0).withKD(0)
            .withKS(0.005).withKV(0.15).withKA(0.01);
        
            // Closed-loop output = how the motor controller applies the calculated control (voltage vs other methods)
        private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
        private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

        // Slip current = The stator current at which the wheels start to slip (not gripping the floor)
        // TUNING: Increase if wheels slip during normal driving, decrease if odometry drifts during acceleration
        private static final Current kSlipCurrent = Amps.of(80.0);
        
        // Motor arrangements = physical setup (integrated = motor and encoder are in one unit)
        private static final DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated;
        private static final SteerMotorArrangement kSteerMotorType = SteerMotorArrangement.TalonFX_Integrated;
        
        // Feedback source = where we get the module's rotation angle from (CANcoder = absolute encoder on module)
        private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

        // Initial configs for the drive and steer motors and the CANcoder; these cannot be null.
        // Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
        // Neutral mode = what happens when motor receives 0% power (Brake = stops, Coast = free-spins)
        // TUNING: Current limits = adjust if motors brown out (lower) or need more power (higher, but watch for brownouts)
        private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
        private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    // Swerve azimuth does not require much torque output, so we can set a relatively low
                    // stator current limit to help avoid brownouts without impacting performance.
                    // Stator current = current through the motor windings (lower limit = less power draw, prevents brownouts)
                    .withStatorCurrentLimit(60)
                    .withStatorCurrentLimitEnable(true));
        // CANcoder = absolute encoder that tells us the exact rotation angle of each swerve module
        private static final CANcoderConfiguration cancoderInitialConfigs = new CANcoderConfiguration();

        // Theoretical free speed (m/s) at 12v applied output;
        // TUNING: This needs to be tuned to your individual robot
        public static final LinearVelocity kSpeedAt12Volts = Units.MetersPerSecond.of(0.0);

        // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
        private static final double kCoupleRatio = 3.125;
        
        // Gear ratio = how many motor rotations = 1 wheel rotation (higher = slower but more torque)
        // NOTE: This assumes you have MK4n swerve modules (narrow) in the front and MK4i swerve modules (wide) in the back
        //private static final double kDriveGearRatio = 5.357142857142857;
        private static final double kDriveGearRatio = 7.13;
        private static final double kSteerGearRatio_MK4i = 150.0/7.0;
        private static final double kSteerGearRatio_MK4n = 18.75; // Narrow

        // The radius of the wheel (in inches) - measure with clamp and calipers, can change through the season
        private static final Distance kWheelRadius = Units.Inches.of(1.9655);

        // These are only used for simulation - adjust if simulation doesn't match real robot
        // Inertia = how much the motor resists changes in speed (higher = slower to change speed)
        // Friction voltage = the voltage required to overcome friction (higher = more voltage required)
        private static final double kSteerInertia = 0.00001;
        private static final double kDriveInertia = 0.001;
        private static final double kSteerFrictionVoltage = 0.25;
        private static final double kDriveFrictionVoltage = 0.25;

        // CAN bus = communication network for motors and sensors (like USB for robot parts)
        private static final String kCANbusName = GeneralConstants.DRIVE_CANIVORE_BUS.getName();
        
        // Pigeon = IMU (Inertial Measurement Unit) that measures robot orientation (yaw/pitch/roll) and rotation rates
        private static final int kPigeonId = 5;
        private static final Pigeon2Configuration pigeonConfigs = new Pigeon2Configuration();
        //private static final MountPoseConfigs mountPose = new MountPoseConfigs();
        //mountPose.withMountPoseYaw(180);
        //pigeonConfigs.withMountPose(mountPose);

        // Drivetrain configuration: combines CAN bus, IMU, and autonomous path planning settings
        public static final BreakerSwerveDrivetrainConstants DRIVETRAIN_CONSTANTS = new BreakerSwerveDrivetrainConstants()
            .withCANBusName(kCANbusName)
            .withPigeon2Id(kPigeonId)
            .withPigeon2Configs(pigeonConfigs)
            .withChoreoConfig(AutoConstants.CHOREO_CONFIG)
            .withPathplannerConfig(AutoConstants.PATHPLANNER_CONFIG);

        // Module factories: creates swerve module constants using shared physical and control parameters
        // MK4n (narrow) modules factory - used for front-left and front-right modules
        private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> MK4nConstantCreator = new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
            .withDriveMotorGearRatio(kDriveGearRatio)
            .withSteerMotorGearRatio(kSteerGearRatio_MK4n)
            .withCouplingGearRatio(kCoupleRatio)
            .withWheelRadius(kWheelRadius)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
            .withSlipCurrent(kSlipCurrent)
            .withSpeedAt12Volts(kSpeedAt12Volts)
            .withDriveMotorType(kDriveMotorType)
            .withSteerMotorType(kSteerMotorType)
            .withFeedbackSource(kSteerFeedbackType)
            .withDriveMotorInitialConfigs(driveInitialConfigs)
            .withSteerMotorInitialConfigs(steerInitialConfigs)
            .withEncoderInitialConfigs(cancoderInitialConfigs)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withSteerFrictionVoltage(kSteerFrictionVoltage)
            .withDriveFrictionVoltage(kDriveFrictionVoltage);
        
        // MK4i (wide) modules factory - used for back-left and back-right modules
        private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> MK4iConstantCreator = new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
            .withDriveMotorGearRatio(kDriveGearRatio)
            .withSteerMotorGearRatio(kSteerGearRatio_MK4i)
            .withCouplingGearRatio(kCoupleRatio)
            .withWheelRadius(kWheelRadius)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
            .withSlipCurrent(kSlipCurrent)
            .withSpeedAt12Volts(kSpeedAt12Volts)
            .withDriveMotorType(kDriveMotorType)
            .withSteerMotorType(kSteerMotorType)
            .withFeedbackSource(kSteerFeedbackType)
            .withDriveMotorInitialConfigs(driveInitialConfigs)
            .withSteerMotorInitialConfigs(steerInitialConfigs)
            .withEncoderInitialConfigs(cancoderInitialConfigs)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withSteerFrictionVoltage(kSteerFrictionVoltage)
            .withDriveFrictionVoltage(kDriveFrictionVoltage);

        // Module inversion flags: corrects wheel encoder directions (not wheel motor directions) for accurate odometry
        // If odometry moves opposite to actual movement, flip these flags
        // These apply to all modules on each side (left = front-left + back-left, right = front-right + back-right)
        // TUNING: Test by moving robot forward - if odometry shows backward movement, flip both flags
        private static final boolean kInvertLeftSide = true;
        private static final boolean kInvertRightSide = false;
        
        // Distance from the robot center line to drive wheel center - get this from the CAD model
        private static final double kBaseModulePosition = 10.875;

        // Individual module configurations

        // ---------------- FRONT LEFT ----------------

        private static final int kFrontLeftDriveMotorId = 8;
        private static final int kFrontLeftSteerMotorId = 9;
        private static final int kFrontLeftEncoderId = 10;
        private static final Angle kFrontLeftEncoderOffset = Rotations.of(0.1904296875); 
        private static final boolean kFrontLeftSteerInvert = true; //true
        private static final boolean kFrontLeftEncoderInvert = false; //false
        private static final Translation2d kFrontLeftModulePosition = new Translation2d(
            Units.Inches.of(kBaseModulePosition),
            Units.Inches.of(kBaseModulePosition));

        // ---------------- FRONT RIGHT ----------------

        private static final int kFrontRightDriveMotorId = 11;
        private static final int kFrontRightSteerMotorId = 12;
        private static final int kFrontRightEncoderId = 13;
        private static final Angle kFrontRightEncoderOffset = Rotations.of(0.3896484375); 
        private static final boolean kFrontRightSteerInvert = true; // true
        private static final boolean kFrontRightEncoderInvert = false; // false
        private static final Translation2d kFrontRightModulePosition = new Translation2d(
            Units.Inches.of(kBaseModulePosition),
            Units.Inches.of(-kBaseModulePosition));

        // ---------------- BACK LEFT ----------------
        
        private static final int kBackLeftDriveMotorId = 14;
        private static final int kBackLeftSteerMotorId = 15;
        private static final int kBackLeftEncoderId = 16;
        private static final Angle kBackLeftEncoderOffset = Rotation.of(-0.047607421875); 
        private static final boolean kBackLeftSteerInvert = true; //true
        private static final boolean kBackLeftEncoderInvert = false; // false
        private static final Translation2d kBackLeftModulePosition = new Translation2d(
            Units.Inches.of(-kBaseModulePosition),
            Units.Inches.of(kBaseModulePosition));

        // ---------------- BACK RIGHT ----------------
        
        private static final int kBackRightDriveMotorId = 17;
        private static final int kBackRightSteerMotorId = 18;
        private static final int kBackRightEncoderId = 19;
        private static final Angle kBackRightEncoderOffset = Rotations.of(-0.217041015625); 
        private static final boolean kBackRightSteerInvert = true; // true
        private static final boolean kBackRightEncoderInvert = false; // false
        private static final Translation2d kBackRightModulePosition = new Translation2d(
            Units.Inches.of(-kBaseModulePosition),
            Units.Inches.of(-kBaseModulePosition));


        // Front modules use MK4n (narrow) gear ratio
        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontLeft = MK4nConstantCreator
            .createModuleConstants(
                kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset,
                kFrontLeftModulePosition.getMeasureX(), kFrontLeftModulePosition.getMeasureY(), kInvertLeftSide,
                kFrontLeftSteerInvert, kFrontLeftEncoderInvert);
        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontRight = MK4nConstantCreator
            .createModuleConstants(
                kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset,
                kFrontRightModulePosition.getMeasureX(), kFrontRightModulePosition.getMeasureY(), kInvertRightSide,
                kFrontRightSteerInvert, kFrontRightEncoderInvert);
        // Back modules use MK4i (wide) gear ratio
        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackLeft = MK4iConstantCreator
            .createModuleConstants(
                kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset,
                kBackLeftModulePosition.getMeasureX(), kBackLeftModulePosition.getMeasureY(), kInvertLeftSide,
                kBackLeftSteerInvert, kBackLeftEncoderInvert);
        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackRight = MK4iConstantCreator
            .createModuleConstants(
                kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset,
                kBackRightModulePosition.getMeasureX(), kBackRightModulePosition.getMeasureY(), kInvertRightSide,
                kBackRightSteerInvert, kBackRightEncoderInvert);
    }


    // ---------------- AUTONOMOUS ----------------
    //
    // PID TUNING NOTES (PathPlanner autos – this is what the auto chooser uses):
    //
    // TRANSLATION (X/Y position following):
    //   • P too low → robot lags behind path, doesn’t reach waypoints.
    //   • P too high → overshoots waypoints, can oscillate.
    //   • D (e.g. 0.5–1.0) → damps oscillation; increase if path following is jerky.
    //   • I → usually 0; add a small value (e.g. 0.01) only if there’s steady-state position error.
    //
    // ROTATION (heading following):
    //   • P too low → robot is slow to correct heading, drifts off path angle.
    //   • P too high → rotation is twitchy or overshoots.
    //   • D → reduces final “jerk” when settling on heading; lower if you see a sharp snap at end of turns.
    //   • I → add a small value (e.g. 0.01–0.03) if rotation consistently settles short (e.g. 87° instead of 90°).
    //
    public static class AutoConstants {

        /** Used by PathPlanner autos (auto chooser).  */
        public static final PIDConstants PATHPLANNER_TRANSLATION_PID = new PIDConstants(2, 0, 0.1); // 7.5,0,0.8
        public static final PIDConstants PATHPLANNER_ROTATION_PID = new PIDConstants(2.0, 0.0, 0.2);//1.5,0.02,0.5
        public static final PathplannerConfig PATHPLANNER_CONFIG = new PathplannerConfig()
            .withTranslationPID(PATHPLANNER_TRANSLATION_PID)
            .withRotationPID(PATHPLANNER_ROTATION_PID);

        /** Used only when running Choreo trajectories (not the PathPlanner auto chooser). */
        public static final PIDConstants TRANSLATION_PID = new PIDConstants(7.5, 0, 0.8);
        public static final PIDConstants ROTATION_PID = new PIDConstants(1.5, 0.02, 0.5);
        public static final ChoreoConfig CHOREO_CONFIG = new ChoreoConfig().withTranslationPID(TRANSLATION_PID)
            .withRotationPID(ROTATION_PID);
    }
}
