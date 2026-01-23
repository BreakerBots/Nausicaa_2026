package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import java.util.Optional;

/**
 * Vision subsystem for handling two LimeLight4 (LL4) cameras for localization.
 */
public class Vision extends SubsystemBase {
    private final Drivetrain drivetrain;
    private final NetworkTable frontCameraData;
    private final NetworkTable backCameraData;
    private final Field2d field;

    // Store latest vision poses for visualization
    private Pose2d fusedPose = null;
    private Pose2d frontCameraPose = null;
    private Pose2d backCameraPose = null;

    // Timer for periodic logging (once per second)
    private double lastLogTime = 0.0;

    // Track measurement acceptance/rejection status for each camera
    private String frontCameraStatus = "No data";
    private String backCameraStatus = "No data";
    private double frontCameraLatency = 0.0;
    private double backCameraLatency = 0.0;
    private double frontCameraTimestamp = 0.0;
    private double backCameraTimestamp = 0.0;

    /** Creates a new Vision subsystem. */
    public Vision(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;

        // Get NetworkTables for both Limelight 4 cameras
        NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
        frontCameraData = ntInstance.getTable(VisionConstants.FRONT_CAMERA);
        backCameraData = ntInstance.getTable(VisionConstants.BACK_CAMERA);

        // Set up the field and start sending its info to Elastic
        field = new Field2d();
        SmartDashboard.putData("Vision/Field", field);

        // Register this subsystem with CommandScheduler so periodic() is called
        CommandScheduler.getInstance().registerSubsystem(this);
    }

    @Override
    public void periodic() {
        // Send robot orientation (IMU data) to Limelights for MegaTag2
        sendRobotOrientationToLimelight(frontCameraData);
        sendRobotOrientationToLimelight(backCameraData);

        // Process vision poses from both cameras
        updatePoseEstimate(frontCameraData, "front_camera");
        updatePoseEstimate(backCameraData, "back_camera");

        // Update Field2d with all poses (fused, and one for each camera) for Elastic
        // dashboard visualization
        fusedPose = drivetrain.getLocalizer().getPose();
        field.getRobotObject().setPose(fusedPose); // Fused pose (odometry + vision)
        if (frontCameraPose != null) {
            field.getObject("front_camera").setPose(frontCameraPose);
        }
        if (backCameraPose != null) {
            field.getObject("back_camera").setPose(backCameraPose);
        }

        // Log vision data once per second
        double currentTime = Timer.getFPGATimestamp();
        if (currentTime - lastLogTime >= 1.0) {
            logVisionData();
            lastLogTime = currentTime;
        }
    }

    /**
     * Robot → Limelight
     * Sends robot IMU orientation data to Limelight for MegaTag2 pose estimation.
     */
    private void sendRobotOrientationToLimelight(NetworkTable cameraData) {
        try {
            // Get Pigeon IMU data
            var pigeon = drivetrain.getPigeon2();
            Rotation3d rotation = pigeon.getRotation3d();

            // Get angular velocities (degrees per second)
            double yawRate = pigeon.getAngularVelocityZWorld().getValueAsDouble();
            double pitchRate = pigeon.getAngularVelocityYWorld().getValueAsDouble();
            double rollRate = pigeon.getAngularVelocityXWorld().getValueAsDouble();

            // Convert rotation to degrees (Limelight expects degrees)
            double yaw = Math.toDegrees(rotation.getZ());
            double pitch = Math.toDegrees(rotation.getY());
            double roll = Math.toDegrees(rotation.getX());

            // Send orientation data to Limelight
            // Format: [yaw, pitch, roll, yawRate, pitchRate, rollRate]
            double[] robotOrientation = { yaw, pitch, roll, yawRate, pitchRate, rollRate };
            cameraData.getEntry("robot_orientation_set").setDoubleArray(robotOrientation);
        } catch (Exception e) {
            // If Pigeon is not available, skip orientation update
        }
    }

    /**
     * Limelight → Robot
     * Reads pose data from a Limelight camera and updates the drivetrain's pose
     * estimate.
     */
    private void updatePoseEstimate(NetworkTable cameraData, String cameraName) {
        // Always use blue alliance coordinate system
        // PathPlanner and other systems will handle alliance flipping internally
        // The "orb" (orientation-based robot pose) here means we're using MegaTag2
        String poseEntryName = "botpose_orb_wpiblue";

        // Get pose data array from Limelight
        // Format: [x, y, z, roll, pitch, yaw, latency, tagCount]
        double[] botpose = cameraData.getEntry(poseEntryName).getDoubleArray(new double[0]);

        // Check if we have valid pose data
        if (botpose.length < 8) {
            logRejection(cameraName, "Invalid data (array length < 8)");
            return; // Invalid data
        }

        double x = botpose[0];
        double y = botpose[1];
        double yaw = botpose[5];
        double latency = botpose[6]; // milliseconds
        double tagCount = botpose[7];

        double timestampSeconds = Timer.getFPGATimestamp() - (latency / 1000.0);

        // Create pose from vision data (using x, y, and yaw)
        Pose2d visionPose = new Pose2d(x, y, Rotation2d.fromDegrees(yaw));

        // Store vision pose for visualization
        if (cameraName.equals("front_camera")) {
            frontCameraPose = visionPose;
            frontCameraLatency = latency;
            frontCameraTimestamp = timestampSeconds;
        } else {
            backCameraPose = visionPose;
            backCameraLatency = latency;
            backCameraTimestamp = timestampSeconds;
        }

        // Only use vision data if we have enough tags
        if (tagCount < VisionConstants.MIN_TAG_COUNT) {
            logRejection(cameraName,
                    String.format("Insufficient tags (got %.0f, need %d)", tagCount, VisionConstants.MIN_TAG_COUNT));
            return;
        }

        // Reject vision measurements during very fast rotation (angular velocity > 720 deg/s)
        // Vision measurements are unreliable during fast spins
        try {
            var pigeon = drivetrain.getPigeon2();
            double angularVelocityDegPerSec = Math.abs(pigeon.getAngularVelocityZWorld().getValueAsDouble());
            if (angularVelocityDegPerSec > 720.0) {
                logRejection(cameraName, String.format("Angular velocity too high (%.1f deg/s > 720 deg/s)", angularVelocityDegPerSec));
                return;
            }
        } catch (Exception e) {
            // If Pigeon is not available, skip angular velocity check
        }

        // Get current pose estimate
        Pose2d currentPose = drivetrain.getLocalizer().getPose();
        double distanceFromOrigin = currentPose.getTranslation().getDistance(new Translation2d(0, 0));

        // If pose is at origin, use resetPose() instead of addVisionMeasurement()
        // resetPose() works immediately without requiring odometry history,
        // whereas addVisionMeasurement() needs odometry buffer entries to fuse with
        if (distanceFromOrigin < 0.1) {
            drivetrain.getLocalizer().resetPose(visionPose);
            return;
        }

        // Not at origin - use addVisionMeasurement() for fusion with existing odometry
        // Check if vision measurement is within reasonable distance of current estimate
        double poseDifference = visionPose.getTranslation().getDistance(currentPose.getTranslation());
        if (poseDifference > VisionConstants.MAX_POSE_DIFFERENCE) {
            logRejection(cameraName, String.format("Pose difference too large (%.2fm > %.2fm)", poseDifference,
                    VisionConstants.MAX_POSE_DIFFERENCE));
            return; // Vision measurement seems unreliable
        }

        // Add vision measurement to pose estimator
        drivetrain.addVisionMeasurement(visionPose, timestampSeconds, VisionConstants.VISION_STD_DEVS);
    }

    /**
     * Calculates the angle error (in radians) to face a target position on the
     * field.
     * Positive = target is to the right, Negative = target is to the left.
     */
    public double getAngleToTarget(Translation2d targetPosition) {
        Pose2d currentPose = drivetrain.getLocalizer().getPose();
        Translation2d robotPosition = currentPose.getTranslation();
        Translation2d toTarget = targetPosition.minus(robotPosition); // vector from robot to target
        Rotation2d desiredHeading = new Rotation2d(toTarget.getX(), toTarget.getY()); // angle to target
        Rotation2d currentHeading = currentPose.getRotation();
        Rotation2d angleError = desiredHeading.minus(currentHeading); // how much we need to rotate
        return angleError.getRadians();
    }

    /**
     * Calculates the distance to a target position on the field.
     */
    public double getDistanceToTarget(Translation2d targetPosition) {
        Pose2d currentPose = drivetrain.getLocalizer().getPose();
        Translation2d robotPosition = currentPose.getTranslation();
        return robotPosition.getDistance(targetPosition);
    }

    /**
     * Checks if any camera detects a tag.
     */
    public boolean isTagDetected() {
        double frontTag = frontCameraData.getEntry("tv").getDouble(0);
        double backTag = backCameraData.getEntry("tv").getDouble(0);
        return frontTag >= 1.0 || backTag >= 1.0;
    }

    /**
     * Gets the ID of the primary tag detected by any camera.
     * Checks both cameras and returns the first tag found.
     */
    public int getDetectedTagId() {
        double[] frontTid = frontCameraData.getEntry("tid").getDoubleArray(new double[0]);
        if (frontTid.length > 0) {
            return (int) frontTid[0];
        }
        double[] backTid = backCameraData.getEntry("tid").getDoubleArray(new double[0]);
        if (backTid.length > 0) {
            return (int) backTid[0];
        }
        return -1; // No tag detected
    }

    /**
     * Gets the field position of an AprilTag by its ID.
     * Uses the official field layout to look up the tag's known position.
     */
    public Translation2d getTagPosition(int tagId) {
        Optional<Pose3d> tagPose = VisionConstants.kAprilTagFieldLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
            return tagPose.get().toPose2d().getTranslation();
        }
        return null;
    }

    /**
     * Gets the field position of the tag currently detected by any camera.
     */
    public Translation2d getDetectedTagPosition() {
        int tagId = getDetectedTagId();
        if (tagId < 0) {
            return null; // No tag detected
        }
        return getTagPosition(tagId);
    }

    /**
     * Calculates the angle error (in radians) to face a specific AprilTag.
     */
    public double getAngleToTag(int tagId) {
        Translation2d tagPosition = getTagPosition(tagId);
        if (tagPosition == null) {
            return 0.0; // Tag not found
        }
        return getAngleToTarget(tagPosition);
    }

    /**
     * Calculates the distance to a specific AprilTag.
     */
    public double getDistanceToTag(int tagId) {
        Translation2d tagPosition = getTagPosition(tagId);
        if (tagPosition == null) {
            return -1.0; // Tag not found
        }
        return getDistanceToTarget(tagPosition);
    }

    /**
     * Logs vision data to both NetworkTables and System.out once per second.
     */
    private void logVisionData() {
        // Get tag IDs from both cameras
        int[] frontTags = getTagIds(frontCameraData);
        int[] backTags = getTagIds(backCameraData);

        // Format poses with 2 decimal places
        String frontPoseStr = formatPose(frontCameraPose);
        String backPoseStr = formatPose(backCameraPose);
        String fusedPoseStr = formatPose(fusedPose);

        // Format tag arrays
        String frontTagsStr = formatTagArray(frontTags);
        String backTagsStr = formatTagArray(backTags);

        // Build log message
        String logMessage = String.format(
                "------------------------------------------------------\n" +
                        "- Front Camera: Tags %s, Pose %s\n" +
                        "- Back Camera: Tags %s, Pose %s\n" +
                        "- Fused: Pose %s",
                frontTagsStr, frontPoseStr,
                backTagsStr, backPoseStr,
                fusedPoseStr);

        // Log to System.out
        System.out.println(logMessage);

        // Log to NetworkTables
        SmartDashboard.putString("Vision/Log", logMessage);
        SmartDashboard.putString("Vision/FrontCamera/Tags", frontTagsStr);
        SmartDashboard.putString("Vision/FrontCamera/Pose", frontPoseStr);
        SmartDashboard.putString("Vision/BackCamera/Tags", backTagsStr);
        SmartDashboard.putString("Vision/BackCamera/Pose", backPoseStr);
        SmartDashboard.putString("Vision/Fused/Pose", fusedPoseStr);

        // Log measurement status and latency/timestamp diagnostics
        SmartDashboard.putString("Vision/FrontCamera/Status", frontCameraStatus);
        SmartDashboard.putString("Vision/BackCamera/Status", backCameraStatus);
        SmartDashboard.putNumber("Vision/FrontCamera/Latency", frontCameraLatency);
        SmartDashboard.putNumber("Vision/BackCamera/Latency", backCameraLatency);
        SmartDashboard.putNumber("Vision/FrontCamera/TimestampAge",
                frontCameraTimestamp > 0 ? Timer.getFPGATimestamp() - frontCameraTimestamp : 0.0);
        SmartDashboard.putNumber("Vision/BackCamera/TimestampAge",
                backCameraTimestamp > 0 ? Timer.getFPGATimestamp() - backCameraTimestamp : 0.0);
    }

    /**
     * Gets tag IDs from a camera's NetworkTable.
     */
    private int[] getTagIds(NetworkTable cameraData) {
        double[] tidArray = cameraData.getEntry("tid").getDoubleArray(new double[0]);
        int[] tagIds = new int[tidArray.length];
        for (int i = 0; i < tidArray.length; i++) {
            tagIds[i] = (int) tidArray[i];
        }
        return tagIds;
    }

    /**
     * Formats a pose to 2 decimal places: (x, y, yaw)
     */
    private String formatPose(Pose2d pose) {
        if (pose == null) {
            return "null";
        }
        double x = Math.round(pose.getX() * 100.0) / 100.0;
        double y = Math.round(pose.getY() * 100.0) / 100.0;
        double yaw = Math.round(pose.getRotation().getDegrees() * 100.0) / 100.0;
        return String.format("(%.2f, %.2f, %.2f)", x, y, yaw);
    }

    /**
     * Formats a tag ID array as a string: [id1, id2, ...]
     */
    private String formatTagArray(int[] tags) {
        if (tags == null || tags.length == 0) {
            return "[]";
        }
        StringBuilder sb = new StringBuilder("[");
        for (int i = 0; i < tags.length; i++) {
            sb.append(tags[i]);
            if (i < tags.length - 1) {
                sb.append(", ");
            }
        }
        sb.append("]");
        return sb.toString();
    }

    /**
     * Logs measurement rejection to NetworkTables.
     */
    private void logRejection(String cameraName, String reason) {
        String keyPrefix = "Vision/" + (cameraName.equals("front_camera") ? "FrontCamera" : "BackCamera")
                + "/Measurement";
        SmartDashboard.putString(keyPrefix + "/Status", "REJECTED: " + reason);

        if (cameraName.equals("front_camera")) {
            frontCameraStatus = "REJECTED: " + reason;
        } else {
            backCameraStatus = "REJECTED: " + reason;
        }
    }
}
