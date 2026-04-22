// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.logging;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.CANBus.CANBusStatus;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.swerve.SwerveModule;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import dev.doglog.DogLog;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.BuildConstants;
import frc.robot.BreakerLib.physics.BreakerVector2;
import frc.robot.BreakerLib.physics.BreakerVector3;
import frc.robot.BreakerLib.physics.ChassisAccels;
import frc.robot.BreakerLib.util.BreakerLibVersion;


/**
* BreakerLog provides comprehensive logging for FRC robots with real-time dashboard support.
*
* This class extends DogLog to add FRC-specific logging capabilities:
*
* LOGGING DESTINATIONS:
* - Files: All logs are written to files via DogLog (always enabled)
* - NetworkTables: Real-time logs published to dashboards (filtered by FMS status)
*
* FMS FILTERING:
* - During matches (FMS connected): NetworkTables publishing disabled to conserve bandwidth
* - During practice (FMS not connected): NetworkTables publishing enabled for real-time monitoring
*
* VERBOSE LOGGING CONTROL:
* - Use setVerboseLogging(true/false) to control high-frequency logging
* - When false: Reduces noise from frequent telemetry (joysticks, sensors, etc.)
* - When true: Enables detailed debugging information
* - Default: false (set in RobotContainer based on match status)
*
* THROTTLED LOGGING:
* - Use log(key, value, true) to throttle high-frequency logs to twice per second
* - When the throttle flag is true, only the first log per key per 0.5s is forwarded
*/
public class BreakerLog extends DogLog implements Subsystem {
    private static ArrayList<CANBus> loggedCANBuses = new ArrayList<>();
    private static final Map<String, EnergyTracker> trackedMotorEnergy = new ConcurrentHashMap<>();

    /** Global verbose logging control - when false, high-frequency logging is disabled */
    private static boolean verboseLogging = true;

    /** Interval (seconds) for throttled logs - limits to 2 Hz per key */
    private static final double THROTTLE_INTERVAL_SEC = 0.5;
    private static final Map<String, Double> lastThrottledLogTime = new ConcurrentHashMap<>();
    private static double lastEnergySummaryLoggedAtSec = Double.NEGATIVE_INFINITY;

    /** Tracks cumulative motor energy for a given motor. */
    private static class EnergyTracker {
        private double cumulativeJoules = 0.0;
        private double lastSampleAtSec = Double.NaN;
        private double lastLoggedAtSec = Double.NEGATIVE_INFINITY;
        private boolean wasEnabledLastCall = false;

        private EnergyTracker() {}
    }

    private BreakerLog() {
        CommandScheduler.getInstance().registerSubsystem(this);
    }

    @Override
    public void periodic() {
        BreakerLog.periodicLog();
    }


    /**
     * Log levels for different types of messages, allows for filtering in dashboards
     */
    public enum LogLevel {
        DEBUG, // Granular details most useful when troubleshooting
        INFO, // Important details
        WARNING, // Not an error, but should be noted
        ERROR // Something went wrong
    }
    
    /* We usually want to log a message with a level */
    public static void log(LogLevel level, String key, String message) {
        log(level.name() + "/" + key, message);
    }

    /**
     * Log a value with optional throttling. When throttle is true, only logs at most twice per second per key.
     * Use for high-frequency periodic logs (e.g. subsystem status) to reduce dashboard load.
     */
    public static void log(String key, Object value, boolean throttle) {
        if (!throttle) {
            logToParent(key, value);
            return;
        }
        double now = Timer.getFPGATimestamp();
        Double last = lastThrottledLogTime.get(key);
        if (last == null || now - last >= THROTTLE_INTERVAL_SEC) {
            lastThrottledLogTime.put(key, now);
            logToParent(key, value);
        }
    }

    private static void logToParent(String key, Object value) {
        if (value instanceof String s) {
            DogLog.log(key, s);
        } else if (value instanceof Boolean b) {
            DogLog.log(key, b);
        } else if (value instanceof Number n) {
            DogLog.log(key, n.doubleValue());
        } else {
            DogLog.log(key, value == null ? "null" : value.toString());
        }
    }

    public static void log(String key, Measure<?> value) {
        log(key + "/Value", value.magnitude());
        log(key + "/Units", value.unit().toString());
    }

    
    public static void log(String key, BreakerVector2 value) {
        log(key + "/X", value.getX());
        log(key + "/Y", value.getY());
        log(key + "/Angle", value.getAngle());
    }

    public static void log(String key, BreakerVector3 value) {
        log(key + "/X", value.getX());
        log(key + "/Y", value.getY());
        log(key + "/Z", value.getZ());
        log(key + "/Angle", value.getAngle());
    }

    public static void log(String key, ChassisAccels value) {
        log(key + "/X", value.getX());
        log(key + "/Y", value.getY());
        log(key + "/Alpha", value.getAlpha());
    }

    public static void log(String key, Trajectory<SwerveSample> value) {
        log(key + "/Poses", value.getPoses());
        log(key + "/InitialSample", value.getInitialSample(false).orElse(new SwerveSample(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, new double[4], new double[4])));
        log(key + "/FinalSample", value.getFinalSample(false).orElse(new SwerveSample(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, new double[4], new double[4])));
        log(key + "/TotalTime", value.getTotalTime());
    }

    public static void log(String key, SwerveSample value) {
        log(key + "/Pose", new Pose2d(value.x, value.y, Rotation2d.fromRadians(value.heading)));
        log(key + "/ChassisSpeeds", new ChassisSpeeds(value.vx, value.vy, value.omega));
        log(key + "/ChassisAccels", new ChassisAccels(value.ax, value.ay, value.alpha));
        log(key + "/Timestamp", value.t);
    }

    public static void log(String key, TalonFXS value) {
        log(key + "/StatorCurrent", value.getStatorCurrent().getValueAsDouble());
        log(key + "/SupplyCurrent", value.getSupplyCurrent().getValueAsDouble());
        log(key + "/Position", value.getPosition().getValueAsDouble());
        log(key + "/Velocity", value.getVelocity().getValueAsDouble());
    }

    public static void log(String key, TalonFX value) {
        log(key + "/StatorCurrent", value.getStatorCurrent().getValueAsDouble());
        log(key + "/SupplyCurrent", value.getSupplyCurrent().getValueAsDouble());
        log(key + "/Position", value.getPosition().getValueAsDouble());
        log(key + "/Velocity", value.getVelocity().getValueAsDouble());
    }

    /**
     * Logs cumulative motor energy always, and detailed TalonFX telemetry when verbose logging is enabled.
     */
    public static void log(String key, TalonFX value, boolean throttle) {
        if (isVerboseLogging()) {
            log(key, value);
        }
        logCumulativeEnergy(key, value, throttle);
        logEnergySummary(throttle);
    }

    public static void log(String key, CANcoder value) {
        log(key + "/AbsolutePosition", value.getAbsolutePosition().getValueAsDouble());
        log(key + "/PositionSinceBoot", value.getPositionSinceBoot().getValueAsDouble());
        log(key + "/Velocity", value.getVelocity().getValueAsDouble());
    }

    public static void log(String key, Pigeon2 value) {
        Rotation3d rot = value.getRotation3d();
        log(key + "/Gyro/AnglesRad/Yaw", rot.getZ());
        log(key + "/Gyro/AnglesRad/Pitch", rot.getY());
        log(key + "/Gyro/AnglesRad/Roll", rot.getX());
        log(key + "/Gyro/AngleRates/YawRate", Units.degreesToRadians(value.getAngularVelocityZWorld().getValueAsDouble()));
        log(key + "/Gyro/AngleRates/PitchRate", Units.degreesToRadians(value.getAngularVelocityYWorld().getValueAsDouble()));
        log(key + "/Gyro/AngleRates/RollRate", Units.degreesToRadians(value.getAngularVelocityXWorld().getValueAsDouble()));
        log(key + "/Accelerometer/X", value.getAccelerationX().getValueAsDouble());
        log(key + "/Accelerometer/Y", value.getAccelerationY().getValueAsDouble());
        log(key + "/Accelerometer/Z", value.getAccelerationZ().getValueAsDouble());
    }

    public static void log(String key, SwerveModule<TalonFX, TalonFX, CANcoder> value) {
        log(key + "/DriveMotor", value.getDriveMotor());
        log(key + "/SteerMotor", value.getSteerMotor());
        log(key + "/SteerEncoder", value.getEncoder());
    }

    @SafeVarargs
    public static void log(String key, SwerveModule<TalonFX, TalonFX, CANcoder>... value) {
        for (int i = 0; i < value.length; i++) {
            log(key + "/" + i, value[i]);
        }
    }

    public static void log(String key, CANBus value) {
        log(key + "/Name", value.getName());
        log(key + "/IsNetworkFD", value.isNetworkFD());
        CANBusStatus status = value.getStatus();
        log(key + "/Status/BusUtilization", status.BusUtilization);
        log(key + "/Status/BusOffCount", status.BusOffCount);
        log(key + "/Status/ReceiveErrorCount", status.REC);
        log(key + "/Status/TransmitErrorCount", status.TEC);
        log(key + "/Status/TransmitBufferFullCount", status.TxFullCount);
    }

    public static void log(String key, Alert value) {
        log(key + "/IsActive", value.get());
        log(key + "/Text", value.getText());
        log(key + "/Type", value.getType());
    }

    public static void log(String key, ProfiledPIDController value) {
        log(key + "/PositionError", value.getPositionError());
        log(key + "/VelocityError", value.getVelocityError());
        log(key + "/SetPosition", value.getSetpoint().position);
        log(key + "/SetVelocity", value.getSetpoint().velocity);
    }

    public static void addCANBus(CANBus value) {
        loggedCANBuses.add(value);
        
    }

    /**
     * Logs cumulative motor electrical energy in Joules since the most recent enable transition.
     * Key should be full path prefix (e.g. "Electrical/Intake/roller").
     */
    private static void logCumulativeEnergy(String key, TalonFX motor, boolean throttle) {
        EnergyTracker tracker = trackedMotorEnergy.computeIfAbsent(key, unused -> new EnergyTracker());
        double nowSec = Timer.getFPGATimestamp();
        boolean enabled = DriverStation.isEnabled();

        // First call for this key: seed timestamp so dt starts from "now" instead of NaN/garbage.
        if (Double.isNaN(tracker.lastSampleAtSec)) {
            tracker.lastSampleAtSec = nowSec;
        }

        // Disable -> enable transition means "new match", so zero cumulative energy for this motor.
        if (enabled && !tracker.wasEnabledLastCall) {
            tracker.cumulativeJoules = 0.0;
            tracker.lastSampleAtSec = nowSec;
            tracker.lastLoggedAtSec = Double.NEGATIVE_INFINITY;
        }

        double dtSec = Math.max(0.0, nowSec - tracker.lastSampleAtSec);
        tracker.lastSampleAtSec = nowSec;

        if (enabled && dtSec > 0.0) {
            double supplyVoltage = motor.getSupplyVoltage().getValueAsDouble();
            double supplyCurrent = motor.getSupplyCurrent().getValueAsDouble();
            double powerWatts = Math.max(0.0, supplyVoltage * supplyCurrent);
            tracker.cumulativeJoules += powerWatts * dtSec;
        }

        tracker.wasEnabledLastCall = enabled;

        String cumulativeKey = key + "CumulativeJoules";
        if (!throttle) {
            log(cumulativeKey, tracker.cumulativeJoules);
            return;
        }

        // Match the standard BreakerLog throttle cadence (2 Hz, twice per second).
        if (nowSec - tracker.lastLoggedAtSec >= THROTTLE_INTERVAL_SEC) {
            log(cumulativeKey, tracker.cumulativeJoules);
            tracker.lastLoggedAtSec = nowSec;
        }
    }

    /** Pull subsystem name from keys in form Electrical/<Subsystem>/<Motor...>. */
    private static String getSubsystemFromElectricalKey(String motorKey) {
        String[] pathParts = motorKey.split("/");
        if (pathParts.length >= 2) {
            return pathParts[1];
        }
        return "Unknown";
    }

    /** Logs robot-wide/subsystem totals and percent-of-total derived from per-motor cumulative Joules. */
    private static void logEnergySummary(boolean throttle) {
        double nowSec = Timer.getFPGATimestamp();
        if (throttle && nowSec - lastEnergySummaryLoggedAtSec < THROTTLE_INTERVAL_SEC) {
            return;
        }
        lastEnergySummaryLoggedAtSec = nowSec;

        Map<String, Double> subsystemTotalsJoules = new HashMap<>();
        double robotTotalJoules = 0.0;

        for (Map.Entry<String, EnergyTracker> trackedMotor : trackedMotorEnergy.entrySet()) {
            String motorKey = trackedMotor.getKey();
            double motorJoules = trackedMotor.getValue().cumulativeJoules;
            robotTotalJoules += motorJoules;
            subsystemTotalsJoules.merge(getSubsystemFromElectricalKey(motorKey), motorJoules, Double::sum);
        }

        log("Electrical/Summary/CumulativeJoules", robotTotalJoules);

        for (Map.Entry<String, EnergyTracker> trackedMotor : trackedMotorEnergy.entrySet()) {
            String motorKey = trackedMotor.getKey();
            double motorJoules = trackedMotor.getValue().cumulativeJoules;
            double motorPercentOfTotal = robotTotalJoules > 0.0 ? (100.0 * motorJoules / robotTotalJoules) : 0.0;
            log(motorKey + "PercentOfTotal", motorPercentOfTotal);
        }

        for (Map.Entry<String, Double> subsystemTotal : subsystemTotalsJoules.entrySet()) {
            String subsystem = subsystemTotal.getKey();
            double subsystemJoules = subsystemTotal.getValue();
            double subsystemPercentOfTotal = robotTotalJoules > 0.0 ? (100.0 * subsystemJoules / robotTotalJoules) : 0.0;
            log("Electrical/Summary/Subsystem/" + subsystem + "/CumulativeJoules", subsystemJoules);
            log("Electrical/Summary/Subsystem/" + subsystem + "/PercentOfTotal", subsystemPercentOfTotal);
        }
    }

    private static void logCANBuses() {{
        for (CANBus bus: loggedCANBuses) 
            log("SystemStats/CanivoreBuses/" + bus.getName(), bus);
        }
    }

    private static void periodicLog() {
        // Only log CAN buses if both DogLog extras are enabled AND our verbose flag is true
        if (options.logExtras() && verboseLogging) {
            logCANBuses();
        }
    }
    
    public static void logMetadata(String key, String value) {
        log("/Metadata/" + key, value);
    }

    public static void logMetadata(Metadata metadata) {
        logMetadata("RobotName", metadata.robotName);
        logMetadata("ProjectYear", Integer.toString(metadata.year));
        logMetadata("Authors", metadata.authors);
        logMetadata("WPILibVersion", metadata.wpiLibVersion);
        logMetadata("BreakerLibVersion", metadata.breakerLibVersion);
        logMetadata("MavenName", metadata.mavenName);
        logMetadata("GitRevision", Integer.toString(metadata.gitRevision));
        logMetadata("GitSHA", metadata.gitSHA);
        logMetadata("GitDate", metadata.gitDate);
        logMetadata("GitBranch", metadata.gitBranch);
        logMetadata("BuildDate", metadata.buildDate);
        logMetadata("Dirty", Integer.toString(metadata.dirty));
    }

    public record Metadata(
        String robotName,
        int year,
        String authors,
        String wpiLibVersion,
        String breakerLibVersion,
        String mavenName,
        int gitRevision,
        String gitSHA,
        String gitDate,
        String gitBranch,
        String buildDate,
        int dirty
    ) {
        public Metadata(
            String robotName, 
            int year, 
            String authors,
            GitInfo gitInfo
        ) {
            this(robotName, year, authors, WPILibVersion.Version, BreakerLibVersion.version, BuildConstants.MAVEN_NAME, gitInfo.gitRevision, gitInfo.gitSHA, gitInfo.gitDate, gitInfo.gitBranch, gitInfo.buildDate, gitInfo.dirty);
        }
    }

    public record GitInfo (String mavenName,
                           int gitRevision,
                           String gitSHA,
                           String gitDate,
                           String gitBranch,
                           String buildDate,
                           int dirty) {
    }

    public static void updateDynamicPublishNT() {
        boolean shouldPub = DriverStation.isDSAttached() && !DriverStation.isFMSAttached();
        setOptions(options.withNtPublish(shouldPub));
    }


    public static void setVerboseLogging(boolean verbose) {
        verboseLogging = verbose;
        
        // Configure DogLog options based on verbose setting
        if (verbose) {
            // Enable all logging features when verbose
            setOptions(options.withLogExtras(true));
        } else {
            // Disable extra logging features when not verbose
            setOptions(options.withLogExtras(false));
        }
    }
    
    public static boolean isVerboseLogging() {
        return verboseLogging;
    }
}
