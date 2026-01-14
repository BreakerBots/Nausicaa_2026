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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.BreakerLib.swerve.BreakerSwerveDrivetrain.BreakerSwerveDrivetrainConstants;
import frc.robot.BreakerLib.swerve.BreakerSwerveDrivetrain.BreakerSwerveDrivetrainConstants.ChoreoConfig;
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

    // ---------------- ARM ----------------

    public static class ArmConstants {
        // Test arm
        public static final Rotation2d POSITION_LEFT = Rotation2d.fromRotations(3); 
        public static final Rotation2d POSITION_RIGHT = Rotation2d.fromRotations(-3); 
        public static final Rotation2d POSITION_MIDDLE = Rotation2d.fromRotations(0); 
        
        // MINNOW ARM
        public static final Rotation2d POSITION_UP = Rotation2d.fromRotations(0); // 0
        public static final Rotation2d POSITION_EXTAKE = Rotation2d.fromRotations(0.2); 
        public static final Rotation2d POSITION_STOW = Rotation2d.fromRotations(-5.0);
        public static final Rotation2d POSITION_DOWN = Rotation2d.fromRotations(-9.2);
        public static final int ARM_MOTOR_ID = 43;
        public static final double ARM_CURRENT_LIMIT = 60.0;
        public static final int BEAM_BREAK_DIO_PORT = 9;

        // ARM TUNING GUIDE - Follow this order for best results:
        // 1. MOTION MAGIC: Start with conservative values, tune for smooth motion profiles
        //    - Cruise velocity (Speed Limit): Start 0.5-1.0, adjust by ±0.25 (increase for faster, decrease if overshooting)
        //    - Acceleration (Speed Change Rate): Start 1.0-2.0, adjust by ±0.5 (increase for snappier, decrease if jerky)
        //    - Jerk (Smoothness Control): Start 5.0-10.0, adjust by ±2.5 (increase for smoother, decrease if too slow)
        // 2. FEEDFORWARD: Tune to reduce PID workload and improve efficiency
        //    - kS (Static Friction): Start 0.05-0.15, adjust by ±0.02 (increase if motor doesn't start moving)
        //    - kG (Gravity Compensation): Start 0.01-0.05, adjust by ±0.01 (increase if arm sags, decrease if overshoots up)
        //    - kV (Velocity Tracking): Start 0.08-0.15, adjust by ±0.02 (increase if too slow, decrease if too fast)
        //    - kA (Acceleration Help): Start 0.005-0.02, adjust by ±0.005 (increase if too slow, decrease if jerky)
        // 3. PID: Fine-tune for accurate positioning and stability
        //    - kP (Position Correction): Start 0.03-0.08, adjust by ±0.01 (increase if slow, decrease if oscillating)
        //    - kI (Steady-State Accuracy): Start 0.001-0.005, adjust by ±0.001 (increase if drifts, decrease if overshooting)
        //    - kD (Oscillation Damping): Start 0.001-0.005, adjust by ±0.001 (increase if oscillating, decrease if jerky)


        // MotionMagic tuning (rotations/second, rotations/second^2, rotations/second^3)
        public static final double MM_CRUISE_VELOCITY = 0.0;   // Speed limit in rotations/second (increase for faster moves, decrease if arm overshoots or oscillates)
        public static final double MM_ACCELERATION = 0.0;       // How quickly arm accelerates to speed limit (increase for snappier response, decrease if jerky or causes overshoot)
        public static final double MM_JERK = 0.05;              // Rate of change of acceleration - smoothness control (increase for smoother motion, decrease if too slow to reach target)

        // FeedForward
        public static final double kS = 0.1;   // Static friction feedforward - minimum voltage to overcome static friction (increase if motor doesn't start moving)
        public static final double kG = 0.02;  // Gravity feedforward - compensates for gravity at different arm positions (increase if arm sags, decrease if it overshoots up)
        public static final double kV = 0.0;  // Velocity feedforward - voltage per unit velocity (increase if arm moves too slow, decrease if too fast)
        public static final double kA = 0.0;  // Acceleration feedforward - voltage per unit acceleration (increase if arm accelerates too slow, decrease if jerky)

        // PID
        public static final double kP = 0.05;  // Proportional gain - how aggressively to correct position errors (increase if slow response, decrease if oscillating)
        public static final double kI = 0.003; // Integral gain - eliminates steady-state error (increase if position drifts, decrease if overshooting)
        public static final double kD = 0.001; // Derivative gain - reduces oscillation and overshoot (increase if oscillating, decrease if jerky)
    }


    // ---------------- ROLLER ----------------
    public static class RollerConstants {
        public static final int ROLLER_MOTOR_ID = 24; 
        public static final double CORAL_EXTAKE_SPEED = 0.15; // -0.5
        public static final double ALGAE_EXTAKE_SPEED = -0.3; // -0.3
        public static final double ALGAE_INTAKE_SPEED = 0.4; // 0.4

        public static final double IDLE_SPEED = 0.0;
        public static final double ROLLER_CURRENT_LIMIT = 60; // 30 for algae stuck
        public static final double ALGAE_STOW_SPEED = 0.0;
    }   

    // ---------------- SWERVE DRIVE ----------------

    public static class DriveConstants {
        public static final AngularVelocity MAXIMUM_MODULE_AZIMUTH_SPEED = Units.DegreesPerSecond.of(720);
        public static final HeadingCompensationConfig HEADING_COMPENSATION_CONFIG = new HeadingCompensationConfig(
            Units.MetersPerSecond.of(0.05),
            Units.RadiansPerSecond.of(0.001),
            Units.Seconds.of(0.2),
            new PIDConstants(1.5, 0, 0));// 1.5
        // public static final SetpointGenerationConfig SETPOINT_GENERATION_CONFIG = new
        // SetpointGenerationConfig(MAXIMUM_MODULE_AZIMUTH_SPEED);
        public static final TeleopControlConfig TELEOP_CONTROL_CONFIG = new TeleopControlConfig();
            // .withHeadingCompensation(HEADING_COMPENSATION_CONFIG);
        // .withSetpointGeneration(SETPOINT_GENERATION_CONFIG);
        public static final LinearVelocity MAXIMUM_TRANSLATIONAL_VELOCITY = Units.MetersPerSecond.of(4.5);
        public static final AngularVelocity MAXIMUM_ROTATIONAL_VELOCITY = Units.RadiansPerSecond.of(9.5);
        // The steer motor uses any SwerveModule.SteerRequestType control request with
        // the output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
        private static final Slot0Configs steerGains = new Slot0Configs()
            .withKP(100).withKI(0).withKD(0.2)
            .withKS(0).withKV(1.5).withKA(0);
        // When using closed-loop control, the drive motor uses the control
        // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
        private static final Slot0Configs driveGains = new Slot0Configs()
            .withKP(0.01).withKI(0).withKD(0)
            .withKS(0.005).withKV(0.15).withKA(0.01);

        // The closed-loop output type to use for the steer motors;
        // This affects the PID/FF gains for the steer motors
        private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
        // The closed-loop output type to use for the drive motors;
        // This affects the PID/FF gains for the drive motors
        private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

        // The stator current at which the wheels start to slip;
        // This needs to be tuned to your individual robot
        private static final Current kSlipCurrent = Amps.of(80.0);

        private static final DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated;
        private static final SteerMotorArrangement kSteerMotorType = SteerMotorArrangement.TalonFX_Integrated;
        private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

        // Initial configs for the drive and steer motors and the CANcoder; these cannot
        // be null.
        // Some configs will be overwritten; check the `with*InitialConfigs()` API
        // documentation.
        private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
        private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    // Swerve azimuth does not require much torque output,
                    // so we can set a relatively low
                    // stator current limit to help avoid brownouts without
                    // impacting performance.
                    .withStatorCurrentLimit(60)
                    .withStatorCurrentLimitEnable(true));
        private static final CANcoderConfiguration cancoderInitialConfigs = new CANcoderConfiguration();
        // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
        private static final Pigeon2Configuration pigeonConfigs = null;

        // Theoretical free speed (m/s) at 12v applied output;
        // This needs to be tuned to your individual robot
        public static final LinearVelocity kSpeedAt12Volts = Units.MetersPerSecond.of(0.0);

        // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
        // This may need to be tuned to your individual robot
        private static final double kCoupleRatio = 3.125;

        private static final double kDriveGearRatio = 5.357142857142857;
        private static final double kSteerGearRatio = 21.428571428571427;
        private static final Distance kWheelRadius = Units.Inches.of(2.0);

        private static final String kCANbusName = GeneralConstants.DRIVE_CANIVORE_BUS.getName();
        private static final int kPigeonId = 5;

        // These are only used for simulation
        private static final double kSteerInertia = 0.00001;
        private static final double kDriveInertia = 0.001;
        // Simulated voltage necessary to overcome friction
        private static final double kSteerFrictionVoltage = 0.25;
        private static final double kDriveFrictionVoltage = 0.25;

        public static final BreakerSwerveDrivetrainConstants DRIVETRAIN_CONSTANTS = new BreakerSwerveDrivetrainConstants()
            .withCANBusName(kCANbusName)
            .withPigeon2Id(kPigeonId)
            .withPigeon2Configs(pigeonConfigs)
            .withChoreoConfig(AutoConstants.CHOREO_CONFIG);

        private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator = new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
            .withDriveMotorGearRatio(kDriveGearRatio)
            .withSteerMotorGearRatio(kSteerGearRatio)
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

        private static final boolean kInvertLeftSide = false;
        private static final boolean kInvertRightSide = true;
        
        // Front Left
        private static final int kFrontLeftDriveMotorId = 10;
        private static final int kFrontLeftSteerMotorId = 11;
        private static final int kFrontLeftEncoderId = 50;
        private static final Angle kFrontLeftEncoderOffset = Rotations.of(0.20751953125);
        private static final boolean kFrontLeftSteerInvert = true;
        private static final boolean kFrontLeftEncoderInvert = false;
        private static final Translation2d kFrontLeftModulePosition = new Translation2d(
            Units.Inches.of(11.25),
            Units.Inches.of(11.25 ));

        // Front Right
        private static final int kFrontRightDriveMotorId = 12;
        private static final int kFrontRightSteerMotorId = 13;
        private static final int kFrontRightEncoderId = 21;
        private static final Angle kFrontRightEncoderOffset = Rotations.of(-0.32177734375);
        private static final boolean kFrontRightSteerInvert = true;
        private static final boolean kFrontRightEncoderInvert = false;

        private static final Translation2d kFrontRightModulePosition = new Translation2d(
            Units.Inches.of(11.25),
            Units.Inches.of(-11.25));

        // Back Left
        private static final int kBackLeftDriveMotorId = 14;
        private static final int kBackLeftSteerMotorId = 15;
        private static final int kBackLeftEncoderId = 22;
        private static final Angle kBackLeftEncoderOffset = Rotation.of(0.12451171875);
        private static final boolean kBackLeftSteerInvert = true;
        private static final boolean kBackLeftEncoderInvert = false;

        private static final Translation2d kBackLeftModulePosition = new Translation2d(
            Units.Inches.of(-11.25),
            Units.Inches.of(11.25));

        // Back Right
        private static final int kBackRightDriveMotorId = 16;
        private static final int kBackRightSteerMotorId = 17;
        private static final int kBackRightEncoderId = 23;
        private static final Angle kBackRightEncoderOffset = Rotations.of(-0.496826171875);
        private static final boolean kBackRightSteerInvert = true;
        private static final boolean kBackRightEncoderInvert = false;
        private static final Translation2d kBackRightModulePosition = new Translation2d(
            Units.Inches.of(-11.25),
            Units.Inches.of(-11.25));

        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontLeft = ConstantCreator
            .createModuleConstants(
                kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset,
                kFrontLeftModulePosition.getMeasureX(), kFrontLeftModulePosition.getMeasureY(), kInvertLeftSide,
                kFrontLeftSteerInvert, kFrontLeftEncoderInvert);
        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontRight = ConstantCreator
            .createModuleConstants(
                kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset,
                kFrontRightModulePosition.getMeasureX(), kFrontRightModulePosition.getMeasureY(), kInvertRightSide,
                kFrontRightSteerInvert, kFrontRightEncoderInvert);
        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackLeft = ConstantCreator
            .createModuleConstants(
                kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset,
                kBackLeftModulePosition.getMeasureX(), kBackLeftModulePosition.getMeasureY(), kInvertLeftSide,
                kBackLeftSteerInvert, kBackLeftEncoderInvert);
        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackRight = ConstantCreator
            .createModuleConstants(
                kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset,
                kBackRightModulePosition.getMeasureX(), kBackRightModulePosition.getMeasureY(), kInvertRightSide,
                kBackRightSteerInvert, kBackRightEncoderInvert);
    }

    public static class AutoConstants {
        public static final PIDConstants TRANSLATION_PID = new PIDConstants(7.5, 0, 0.8);
        public static final PIDConstants ROTATION_PID = new PIDConstants(1.5, 0, 1);
        public static final ChoreoConfig CHOREO_CONFIG = new ChoreoConfig().withTranslationPID(TRANSLATION_PID)
            .withRotationPID(ROTATION_PID);
    }
}
