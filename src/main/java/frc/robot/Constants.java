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
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
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
        public static final int kDriverControllerPort2 = 1;
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

        /** Std dev calculation: true = distance-threshold approach (single/multi-tag bases, quadratic distance), false = trust-score approach. */
        public static final boolean USE_DYNAMIC_STD_DEVS_V2 = true;

        // Limelight 4 camera names (configured in Limelight UI)
        public static final String FRONT_CAMERA = "limelight-f";
        public static final String BACK_LEFT_CAMERA = "limelight-bl";
        public static final String BACK_RIGHT_CAMERA = "limelight-br";
        
        // Camera pose relative to robot center (meters, degrees)
        // Format: [forward, side, up, roll, pitch, yaw]
        // public static final double[] FRONT_CAMERA_POSE = {0.1513179, 0.0, 0.128, 0.0, 26.0, 0};
        // public static final double[] BACK_LEFT_CAMERA_POSE = {-0.30145941, -0.29537006, 0.187641611, 0.0, 28.1, -145.0093};
        // public static final double[] BACK_RIGHT_CAMERA_POSE = {-0.30441567, 0.24862055, 0.1876415856, 0.0, 28.1, 144};
        public static final double[] FRONT_CAMERA_POSE = {0.1602181454, 0.0002463292, 0.4868479214, 0.0, 26.0, 0};
        public static final double[] BACK_LEFT_CAMERA_POSE = {-0.3057986264, -0.2792043692, 0.1893890548, 0.0, 28.1, -141};
        public static final double[] BACK_RIGHT_CAMERA_POSE = {-0.2991690994, 0.2016386874, 0.2190634684, 0.0, 28.1, 141};
                
        
        // Minimum number of tags required to trust a vision measurement
        public static final int MIN_TAG_COUNT = 1;
        
        // Maximum pose difference from current estimate to accept vision measurement (meters)
        public static final double MAX_POSE_DIFFERENCE = 6.0;

        // Vision measurement standard deviations (x, y, theta)
        // Increase these values to trust vision measurements less
        // Units: meters for x/y, radians for theta
        // Theta: 9999999 to fully trust IMU for rotation (vision won't correct heading)
        public static final Matrix<N3, N1> VISION_STD_DEVS = 
            VecBuilder.fill(0.15, 0.15, 99999999);        
        
        // Dynamic standard deviation scaling factors (tag count + proximity)
        // trustScore = tagCount * TAG_COUNT_SCALE_FACTOR + PROXIMITY_SCALE_FACTOR / (1 + avgTagDist)
        // stdDev = baseStdDev / (1 + trustScore)  --> higher trustScore = more trust = lower std dev
        // More tags and closer tags both increase trust. A camera with 1 close tag can beat 2 far tags.
        public static final double TAG_COUNT_SCALE_FACTOR = 0.5;
        /** How much proximity boosts trust. avgTagDist is in meters. At 1m: 1/(1+1)=0.5 boost. At 4m: 1/5=0.2 boost. */
        public static final double PROXIMITY_SCALE_FACTOR = 2.0;

        /** V2 approach: single-tag base std devs (higher = less trusted). */
        public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(3.5, 3.5, 10.0);
        /** V2 approach: multi-tag base std devs (2+ tags). */
        public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1.0);
        /** V2 approach: max avg distance (m) for single-tag before rejecting. */
        public static final double MAX_SINGLE_TAG_DIST_METERS = 4.5;
        /** V2 approach: max avg distance (m) for multi-tag before rejecting. */
        public static final double MAX_MULTI_TAG_DIST_METERS = 6.5;
        /** V2 approach: distance² scaling. stdDev *= 1 + (avgDist² / this). */
        public static final double DISTANCE_SCALE_FACTOR = 5.0;
    }


    // --------------- FIELD --------------

    public static class FieldConstants {

        public static final double NZ_MIN_X = 4.625594;
        public static final double NZ_MAX_X = 11.915394;
        public static final double MIDPOINT_Y = 4.034536;

        /** Max distance (m) we allow for driver-initiated navigate-to-trench moves. */
        public static final double NAVIGATE_TO_TRENCH_MAX_DISTANCE_METERS = 3.0;

        // Target pose for tele-op "navigateToPoint" commands (via PathPlanner on-the-fly).
        
        public static final Pose2d POSE_BLUE_LEFT_CLIMBING_TOWER = new Pose2d(1.05, 4.75, Rotation2d.fromDegrees(180));
        public static final Pose2d POSE_RED_LEFT_CLIMBING_TOWER = new Pose2d(15.49, 3.32, Rotation2d.fromDegrees(0));
        
        public static final Pose2d POSE_BLUE_LEFT_EXIT_AZ_VIA_TRENCH = new Pose2d(3.3, 7.408, Rotation2d.fromDegrees(0));
        public static final Pose2d POSE_BLUE_RIGHT_EXIT_AZ_VIA_TRENCH = new Pose2d(3.3, 0.634, Rotation2d.fromDegrees(0));
        
        // We may not need these -- PathPlanner might automatically flip them
        public static final Pose2d POSE_RED_LEFT_EXIT_AZ_VIA_TRENCH = new Pose2d(13.24, 0.634, Rotation2d.fromDegrees(180));
        public static final Pose2d POSE_RED_RIGHT_EXIT_AZ_VIA_TRENCH = new Pose2d(13.24, 7.408, Rotation2d.fromDegrees(180));

        // Targets to aim at for shooting/passing.

        public static final Translation2d TARGET_BLUE_HUB_CENTER = new Translation2d(4.618, 4.036);
        public static final Translation2d TARGET_BLUE_AZ_LEFT = new Translation2d(1.5, 6.5);
        public static final Translation2d TARGET_BLUE_AZ_RIGHT = new Translation2d(1.5, 1.5);

        public static final Translation2d TARGET_RED_HUB_CENTER = new Translation2d(11.915, 4.036);
        public static final Translation2d TARGET_RED_AZ_LEFT = new Translation2d(15.0, 1.5);
        public static final Translation2d TARGET_RED_AZ_RIGHT = new Translation2d(15.0, 6.5);   
        
        public static final int HUB_TAG_ID_RED = 10;
        public static final int HUB_TAG_ID_BLUE = 26;
        public static final int TRENCH_TAG_ID_RED = 12;
        public static final int TRENCH_TAG_ID_BLUE = 28;

        /** Returns the hub AprilTag ID for the current alliance (red or blue). Defaults to blue when alliance is not assigned. */
        public static int getHubTagID() {
            if (isRedAlliance()) {
                return HUB_TAG_ID_RED;
            } else {
                return HUB_TAG_ID_BLUE;
            }
        }

        public static int getTrenchTagID() {
            if (isRedAlliance()) {
                return TRENCH_TAG_ID_RED;
            } else {
                return TRENCH_TAG_ID_BLUE;
            }
        }

        /** Returns the hub center point (x, y) for the current alliance. */
        public static Translation2d getTargetHubCenter() {
            if (isRedAlliance()) {
                return TARGET_RED_HUB_CENTER;
            } else {
                return TARGET_BLUE_HUB_CENTER;
            }
        }

        public static Pose2d getTargetClimbingPose() {
            if (isRedAlliance()) {
                return POSE_RED_LEFT_CLIMBING_TOWER;
            } else {
                return POSE_BLUE_LEFT_CLIMBING_TOWER;
            }
        }

        public static Pose2d getTrenchLeftExitPose() {
            if (isRedAlliance()) {
                return POSE_RED_LEFT_EXIT_AZ_VIA_TRENCH;
            } else {
                return POSE_BLUE_LEFT_EXIT_AZ_VIA_TRENCH;
            }
        }

        public static Pose2d getTrenchRightExitPose() {
            if (isRedAlliance()) {
                return POSE_RED_RIGHT_EXIT_AZ_VIA_TRENCH;
            } else {
                return POSE_BLUE_RIGHT_EXIT_AZ_VIA_TRENCH;
            }
        }

        /** Returns the tracking target for the based on robot pose and alliance.
        * In alliance zone: hub center. Outside AZ: left/right AZ target based on Y. */
        public static Translation2d getTargetForPose(Pose2d pose) {
            double x = pose.getX();
            double y = pose.getY();
            if (isRedAlliance()) {
                if (x >= NZ_MAX_X) return getTargetHubCenter();
                return y > MIDPOINT_Y ? TARGET_RED_AZ_RIGHT : TARGET_RED_AZ_LEFT;
            } else {
                if (x <= NZ_MIN_X) return getTargetHubCenter();
                return y > MIDPOINT_Y ? TARGET_BLUE_AZ_LEFT : TARGET_BLUE_AZ_RIGHT;
            }
        }

        private static Optional<Alliance> getAlliance() {
            Optional<Alliance> alliance = DriverStation.getAlliance();
            return alliance;
        }

        public static boolean isRedAlliance() {
            Optional<Alliance> alliance = getAlliance();
            return alliance.isPresent() && alliance.get() == Alliance.Red;
        }

        /** Returns true if the pose's X coordinate is inside the neutral zone range. */
        public static boolean inNZ(Pose2d pose) {
            double xMeters = pose.getX();
            return xMeters > NZ_MIN_X && xMeters < NZ_MAX_X;
        }
    }


    // --------------- INTAKE --------------

    public static class IntakeConstants {
        public static final int PIVOT_MOTOR_ID = 20;
        public static final int ROLLER_MOTOR_ID = 21;
        public static final int PIVOT_ENCODER_ID = 25;

        // ---------- Pivot ----------

        /** CANcoder: offset so position reads POSITION_STOWED when pivot is physically stowed. 
         * Determine the raw value via Phoenix Tuner.
         * offset = desiredValue - rawValue = 0 - 0.25 = -0.25. */
        public static final double PIVOT_ENCODER_OFFSET_ROTATIONS = 0.224365234375; // Direct copy-paste from mag offsets
        /** Choose a value safely beyond the mechanism's travel 
         * 0.5 is safe for an arm that rotates less than 180 degrees. */
        public static final double PIVOT_ENCODER_DISCONTINUITY = 0.5;

        public static final Rotation2d POSITION_STOWED = Rotation2d.fromRotations(-0.005);  //-0.025
        public static final Rotation2d POSITION_EXTENDED = Rotation2d.fromRotations(-0.28); // Need to Tune
        public static final Rotation2d POSITION_JIGGLE_LOW = Rotation2d.fromRotations(-0.27);
        public static final Rotation2d POSITION_JIGGLE_HIGH = Rotation2d.fromRotations(-0.15);
        //public static final Rotation2d POSITION_JIGGLE_HIGHER = Rotation2d.fromRotations(-0.10);
        

        //Motion Magic
        public static final double PIVOT_MM_CRUISE_VELOCITY = 1.0; // 1.0
        public static final double PIVOT_MM_ACCELERATION = 2.0; // 2.0
        public static final double PIVOT_MM_JERK = 10; // 10

        // Feedforward
        public static final double PIVOT_kS = 0.08;
        public static final double PIVOT_kG = 0.00;
        public static final double PIVOT_kV = 0.12;
        public static final double PIVOT_kA = 0.01;

         // PID
        public static final double PIVOT_kP = 2.0;
        public static final double PIVOT_kI = 0.00;
        public static final double PIVOT_kD = 0.08;

        /** Stator current limit (A) for pivot */
        public static final int PIVOT_STATOR_CURRENT_LIMIT = 70; //70
        public static final int PIVOT_SUPPLY_CURRENT_LIMIT = 60; //60


        // ---------- Roller ----------

        public static final double SPEED_IDLE = 0;
        public static final double SPEED_EXTAKE = 0.4;
        public static final double SPEED_INTAKE = -0.8; 
        public static final double SPEED_FEED_JIGGLE = -0.2;

        // Feedforward 
        public static final double ROLLER_kS = 0.1; // 0.1
        public static final double ROLLER_kV = 0.12; //0.12

        // PID
        public static final double ROLLER_kP = 0.25; //0.25
        public static final double ROLLER_kI = 0.00; //0.0
        public static final double ROLLER_kD = 0.00; //0.0

        /** Used to adjust roller speed based on drivetrain velocity. */
        public static final double ROLLER_DIAMETER_METERS = 1.5 * 0.0254;
        public static final double ROLLER_CIRCUMFERENCE_METERS = ROLLER_DIAMETER_METERS * Math.PI;
        /** Motor pinion : roller gear (teeth). Roller RPS = motor RPS × this ratio. */
        public static final double ROLLER_GEAR_RATIO = 20.0 / 28.0; // wAS 20 / 18
        /** Motor (rotor) rev/s at 100% duty. Converts duty cycle to velocity for closed-loop control. */
        public static final double ROLLER_MOTOR_RPS_AT_FULL_DUTY = 70.0; // Verified via Phoenix Tuner
        // Kraken x60 at full duty = 100 rev/s
        public static final double ROLLER_REV_PER_SEC_AT_FULL_DUTY = ROLLER_MOTOR_RPS_AT_FULL_DUTY * ROLLER_GEAR_RATIO;
        // Static intake (SPEED_INTAKE): |duty| × ROLLER_REV_PER_SEC_AT_FULL_DUTY = roller rev/s.
        // 2 rev per roller circumference while driving: v_max ≈ rollerRps × C / 2.
        // Example: |0.52| duty → ~40 roller rev/s; meets 2 rev per C at ~2.4 m/s forward (see intake tuning).
        
        /** Stator current limit (A) for roller */
        public static final int ROLLER_STATOR_CURRENT_LIMIT = 80;
        public static final int ROLLER_SUPPLY_CURRENT_LIMIT = 60;
    }


    public static class ShooterConstants {

        // --------------- SHOOTER --------------

        public static final int SHOOTER_FLYWHEEL_1_MOTOR_ID = 30;
        public static final int SHOOTER_FLYWHEEL_2_MOTOR_ID = 31;
        public static final int SHOOTER_FLYWHEEL_3_MOTOR_ID = 32;
        
        public static final double ACCELERATION_FLYWHEEL = 20.0; // 200.0

        public static final double SPEED_IDLE = 0;
        public static final double SPEED_FLYWHEEL_ACTIVE = 54.0; // Was 68


        public static final double FLYWHEEL_SPEED_TOLERANCE = 0.05; // Within 5% of target speed
        //public static final double FEED_PAUSE_ANGLE_THRESHOLD_RAD = Math.toRadians(2.5); // Within 2.5 degrees of target heading

        /** Valid distance range (m) for TrajectoryManager lookup; outside this falls back to SPEED_FLYWHEEL_ACTIVE. */
        public static final double SHOOTER_RANGE_MIN = 0.25;
        public static final double SHOOTER_RANGE_MAX = 15.0;

        // Feedforward
        public static final double SHOOTER_kS = 0.1;
        public static final double SHOOTER_kV = 0.12;

        // PID
        public static final double SHOOTER_kP = 0.5; // 0.25
        public static final double SHOOTER_kI = 0.0;
        public static final double SHOOTER_kD = 0.0;

        /** Shooter center offset from robot center: 3" to the right. In robot frame +Y is left, so right = -Y. */
        public static final double SHOOTER_OFFSET_RIGHT_METERS = 4.0 * 0.0254;

        public static final int FLYWHEEL_STATOR_CURRENT_LIMIT = 80; // 80, 40
        public static final int FLYWHEEL_SUPPLY_CURRENT_LIMIT = 50; // 50, 25


        // --------------- HOOD --------------

        public static final int HOOD_MOTOR_ID = 33;
        public static final int HOOD_ENCODER_ID = 35;

        public static final double HOOD_ENCODER_OFFSET_ROTATIONS = 0.01123046875; //0.423095703125 - Direct copy-paste from mag offsets
        /** Choose a value safely beyond the mechanism's travel */
        public static final double HOOD_ENCODER_DISCONTINUITY = 0.5;

        public static final double SPEED_HOOD_UP = -0.2;
        public static final double SPEED_HOOD_DOWN = 0.4;
        public static final double AUTO_SPEED_HOOD_DOWN = -0.3;
        
        public static final double POSITION_HOOD_MIN = 0.000; //-0.002197
        public static final double POSITION_HOOD_MAX = 0.473389; //0.477295
        public static final double POSITION_HOOD_LATCH = 0.101318; // Not used right now
        public static final double POSITION_HOOD_TEST = 0.03;
       
        // Keep push hood until we're within this tolerance
        public static final double HOOD_TARGET_TOLERANCE_ROTATIONS = 0.005;

        // We can start shooting once we reach this tolerance
        public static final double HOOD_GTS_TOLERANCE_ROTATIONS = 0.01;

        /**
         * Max time to hold the start of the shoot sequence while waiting for hood to reach trajectory
         * setpoint (e.g. continuous aim + shoot). After this, spin-up/feed begin anyway so a flaky hood
         * does not block shooting.
         */
        public static final double HOOD_PRESHOOT_MAX_WAIT_SECONDS = 1.2;

        // Motion Magic
        public static final double HOOD_MM_CRUISE_VELOCITY = 0.5;
        public static final double HOOD_MM_ACCELERATION = 1.0;
        public static final double HOOD_MM_JERK = 5.0;

        // Feedforward
        public static final double HOOD_kS = 0.08;
        public static final double HOOD_kG = 0.00;
        public static final double HOOD_kV = 0.12;
        public static final double HOOD_kA = 0.01;

        // PID
        public static final double HOOD_kP = 2.4;
        public static final double HOOD_kI = 0.0;
        public static final double HOOD_kD = 0.0;

        public static final int HOOD_STATOR_CURRENT_LIMIT = 50;
        public static final int HOOD_SUPPLY_CURRENT_LIMIT = 50;
    }

    // --------------- HOPPER --------------

    public static class HopperConstants {
        public static final int HOPPER_MOTOR_ID = 40;
        public static final int FEEDER_MOTOR_ID = 41;
        
        public static final double SPEED_INACTIVE = 0;
        public static final double SPEED_INDEXING = 0.5;
        public static final double SPEED_FEEDING = 0.6;
        public static final double SPEED_UNCLOG_INDEXER = -0.1;
        public static final double SPEED_UNCLOG_FEEDER = -0.12;

        public static final int INDEXER_STATOR_CURRENT_LIMIT = 90; //90
        public static final int INDEXER_SUPPLY_CURRENT_LIMIT = 70; //70
        public static final int FEEDER_STATOR_CURRENT_LIMIT = 90; //90
        public static final int FEEDER_SUPPLY_CURRENT_LIMIT = 70; //70
    }

    // --------------- CLIMB --------------
    // Climb uses setpoint-based control with external encoder (or motor encoder as fallback).

    public static class ClimbConstants {
        public static final int CLIMB_MOTOR_ID = 50;
        public static final int CLIMB_ENCODER_ID = 55;
        // public static final int CLIMB_ENCODER_OFFSET = 0.069580078125;

        public static final double SETPOINT_UP = -1.788125;
        public static final double SETPOINT_CLIMBED = -0.5;
        public static final double SETPOINT_DOWN = 0.1;
        
        public static final double SETPOINT_TOLERANCE = 0.1; // How close is close enough?

        /** Faster speeds for extending/retracting. */
        public static final double SPEED_EXTENDING = 0.5;
        public static final double SPEED_RETRACTING = -0.2;
        
        /** Slower speeds for ascending/descending (0.2 = 20% motor power) */
        public static final double SPEED_ASCENDING = -0.2;
        public static final double SPEED_DESCENDING = 0.2;

        // PID 
        public static final double PID_kP = 4.0;
        public static final double PID_kI = 0.0;
        public static final double PID_kD = 0.0;

        /** Homing: voltage to move climb toward stowed (negative = down toward limit). */
        public static final double HOMING_VOLTAGE = -0.3;
        /** Supply current threshold (A) to detect stall at mechanical limit. */
        public static final Current HOMING_DETECT_CURRENT_THRESHOLD = Amps.of(0.1);
        /** Time (s) current must stay above threshold before accepting stall. */
        public static final double HOMING_STALL_TIME_SECONDS = 0.8;
        /** Homing timeout (s) – bail if stall not detected. */
        public static final double HOMING_TIMEOUT_SECONDS = 5.0;
        /** Stator current limit (A) during homing – loosened so motor can stall without tripping. */
        public static final int HOMING_STATOR_CURRENT_LIMIT = 20;
        //public static final int HOMING_SUPPLY_CURRENT_LIMIT = 15; // idk
        /** Stator current limit (A) for normal operation (restored after homing). Lowered for brownout mitigation. */
        public static final int NORMAL_STATOR_CURRENT_LIMIT = 30;
        //public static final int NORMAL_SUPPLY_CURRENT_LIMIT = 25;

        public static final CurrentLimitsConfigs HOMING_CURRENT_LIMITS = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(HOMING_STATOR_CURRENT_LIMIT)
                //.withSupplyCurrentLimit(HOMING_SUPPLY_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true);
                //.withSupplyCurrentLimitEnable(true);
        public static final CurrentLimitsConfigs NORMAL_CURRENT_LIMITS = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(NORMAL_STATOR_CURRENT_LIMIT)
                //.withSupplyCurrentLimit(NORMAL_SUPPLY_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true);
                //.withSupplyCurrentLimitEnable(true);
    }

    // ---------------- SWERVE DRIVE ----------------

    public static class DriveConstants {
        
        /** ROBOT-LEVEL MAXIMUM SPEEDS - How fast can the robot drive and rotate (currently reduced for testing)
         * Translational = forward/backward and left/right movement (X and Y on the field)
         * Rotational = spinning in place (turning) */
        public static final LinearVelocity MAXIMUM_TRANSLATIONAL_VELOCITY = Units.MetersPerSecond.of(2.4);
        public static final LinearVelocity ALIGN_MODE_MAXIMUM_TRANSLATIONAL_VELOCITY = Units.MetersPerSecond.of(0.4);
        public static final AngularVelocity MAXIMUM_ROTATIONAL_VELOCITY = Units.RadiansPerSecond.of(5.0);
        public static final AngularVelocity ALIGN_MODE_MAXIMUM_ROTATIONAL_VELOCITY = Units.RadiansPerSecond.of(0.8);
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

        public static final TeleopControlConfig TELEOP_CONTROL_CONFIG = new TeleopControlConfig()
            .withHeadingCompensation(HEADING_COMPENSATION_CONFIG);
            // .withSetpointGeneration(SETPOINT_GENERATION_CONFIG);

        /** Max distance (m) for navigateToPose – refuse to pathfind if robot is farther. Prevents accidental long drives. */
        public static final double NAVIGATE_TO_POSE_MAX_DISTANCE_METERS = 5.0;

        public static final double RANGE_TO_TARGET_TOLERANCE = 0.1; // Close enough to target distance, meters
        /** Time (s) to track target before allowing spin-up in shootCommand. */
        public static final double TRACK_BEFORE_SHOOT_TIMEOUT = 1.0;
        public static final double RANGE_TO_TARGET_MAX_DISTANCE = 5.0; // Maximum distance to target, meters
        /** Proportional gain for range-to-target: position error (m) → velocity (m/s). Matches PathPlanner translation for consistency. */
        public static final double RANGE_TO_TARGET_KP = 2.0;
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

        // al configs for the drive and steer motors and the CANcoder; these cannot be null.
        // Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
        // Neutral mode = what happens when motor receives 0% power (Brake = stops, Coast = free-spins)

        // Current limits = adjust if motors brown out (lower) or need more power (higher)
        private static final int DRIVE_STATOR_CURRENT_LIMIT = 90; //90
        private static final int DRIVE_SUPPLY_CURRENT_LIMIT = 80; //80
        private static final int STEER_STATOR_CURRENT_LIMIT = 60; //60
        private static final int STEER_SUPPLY_CURRENT_LIMIT = 50; //60
        
        private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(DRIVE_STATOR_CURRENT_LIMIT)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(DRIVE_SUPPLY_CURRENT_LIMIT)
                    .withSupplyCurrentLimitEnable(true));
        private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(STEER_STATOR_CURRENT_LIMIT)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(STEER_SUPPLY_CURRENT_LIMIT)
                    .withSupplyCurrentLimitEnable(true));
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
        private static final Angle kFrontLeftEncoderOffset = Rotations.of(0.218994140625); 
        private static final boolean kFrontLeftSteerInvert = true; //true
        private static final boolean kFrontLeftEncoderInvert = false; //false
        private static final Translation2d kFrontLeftModulePosition = new Translation2d(
            Units.Inches.of(kBaseModulePosition),
            Units.Inches.of(kBaseModulePosition));

        // ---------------- FRONT RIGHT ----------------

        private static final int kFrontRightDriveMotorId = 11;
        private static final int kFrontRightSteerMotorId = 12;
        private static final int kFrontRightEncoderId = 13;
        private static final Angle kFrontRightEncoderOffset = Rotations.of(-0.306884765625); 
        private static final boolean kFrontRightSteerInvert = true; // true
        private static final boolean kFrontRightEncoderInvert = false; // false
        private static final Translation2d kFrontRightModulePosition = new Translation2d(
            Units.Inches.of(kBaseModulePosition),
            Units.Inches.of(-kBaseModulePosition));

        // ---------------- BACK LEFT ----------------
        
        private static final int kBackLeftDriveMotorId = 14;
        private static final int kBackLeftSteerMotorId = 15;
        private static final int kBackLeftEncoderId = 16;
        private static final Angle kBackLeftEncoderOffset = Rotation.of(-0.237060546875); 
        private static final boolean kBackLeftSteerInvert = true; //true
        private static final boolean kBackLeftEncoderInvert = false; // false
        private static final Translation2d kBackLeftModulePosition = new Translation2d(
            Units.Inches.of(-kBaseModulePosition),
            Units.Inches.of(kBaseModulePosition));

        // ---------------- BACK RIGHT ----------------
        
        private static final int kBackRightDriveMotorId = 17;
        private static final int kBackRightSteerMotorId = 18;
        private static final int kBackRightEncoderId = 19;
        private static final Angle kBackRightEncoderOffset = Rotations.of(-0.12744140625); 
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
