package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.BreakerLib.util.logging.BreakerLog;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import java.util.Optional;

/**
 * Vision subsystem for handling three LimeLight4 (LL4) cameras for localization.
 */
public class Vision extends SubsystemBase {
    private final Drivetrain drivetrain;
    private final Field2d field;

    // Store latest vision poses for visualization
    private Pose2d fusedPose = null;
    private Pose2d frontCameraPose = null;
    private Pose2d backLeftCameraPose = null;
    private Pose2d backRightCameraPose = null;
    
    // Store latest pose estimates for metadata
    private PoseEstimate frontCameraEstimate = null;
    private PoseEstimate backLeftCameraEstimate = null;
    private PoseEstimate backRightCameraEstimate = null;

    // Timer for periodic logging (once per second)
    private double lastLogTime = 0.0;

    // Track measurement acceptance/rejection status for each camera
    private String frontCameraStatus = "No data";
    private String backLeftCameraStatus = "No data";
    private String backRightCameraStatus = "No data";
    private String frontCameraLastRejection = "None";
    private String backLeftCameraLastRejection = "None";
    private String backRightCameraLastRejection = "None";

    /** Creates a new Vision subsystem. */
    public Vision(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;

        // Configure camera poses relative to robot center
        LimelightHelpers.setCameraPose_RobotSpace(
            VisionConstants.FRONT_CAMERA,
            VisionConstants.FRONT_CAMERA_POSE[0], // forward
            VisionConstants.FRONT_CAMERA_POSE[1], // side
            VisionConstants.FRONT_CAMERA_POSE[2], // up
            VisionConstants.FRONT_CAMERA_POSE[3], // roll
            VisionConstants.FRONT_CAMERA_POSE[4], // pitch
            VisionConstants.FRONT_CAMERA_POSE[5]  // yaw
        );
        
        LimelightHelpers.setCameraPose_RobotSpace(
            VisionConstants.BACK_LEFT_CAMERA,
            VisionConstants.BACK_LEFT_CAMERA_POSE[0], // forward
            VisionConstants.BACK_LEFT_CAMERA_POSE[1], // side
            VisionConstants.BACK_LEFT_CAMERA_POSE[2], // up
            VisionConstants.BACK_LEFT_CAMERA_POSE[3], // roll
            VisionConstants.BACK_LEFT_CAMERA_POSE[4], // pitch
            VisionConstants.BACK_LEFT_CAMERA_POSE[5]  // yaw
        );
        LimelightHelpers.setCameraPose_RobotSpace(
            VisionConstants.BACK_RIGHT_CAMERA,
            VisionConstants.BACK_RIGHT_CAMERA_POSE[0], // forward
            VisionConstants.BACK_RIGHT_CAMERA_POSE[1], // side
            VisionConstants.BACK_RIGHT_CAMERA_POSE[2], // up
            VisionConstants.BACK_RIGHT_CAMERA_POSE[3], // roll
            VisionConstants.BACK_RIGHT_CAMERA_POSE[4], // pitch
            VisionConstants.BACK_RIGHT_CAMERA_POSE[5]  // yaw
        );

        // Set up the field and start sending its info to Elastic
        field = new Field2d();
        SmartDashboard.putData("Vision/Field", field);

        // Register this subsystem with CommandScheduler so periodic() is called
        CommandScheduler.getInstance().registerSubsystem(this);
    }

    @Override
    public void periodic() {
        // Send robot orientation (IMU data) to Limelights for MegaTag2 (only needed for MegaTag2)
        if (VisionConstants.USE_MEGATAG2) {
            sendRobotOrientationToLimelight(VisionConstants.FRONT_CAMERA);
            sendRobotOrientationToLimelight(VisionConstants.BACK_LEFT_CAMERA);
            sendRobotOrientationToLimelight(VisionConstants.BACK_RIGHT_CAMERA);
        }

        // Process vision poses from all three cameras
        updatePoseEstimate(VisionConstants.FRONT_CAMERA);
        updatePoseEstimate(VisionConstants.BACK_LEFT_CAMERA);
        updatePoseEstimate(VisionConstants.BACK_RIGHT_CAMERA);

        // Update Field2d with all poses (fused, and one for each camera) for Elastic
        // dashboard visualization
        fusedPose = drivetrain.getLocalizer().getPose();
        field.getRobotObject().setPose(fusedPose); // Fused pose (odometry + vision)
        if (frontCameraPose != null) {
            field.getObject(VisionConstants.FRONT_CAMERA).setPose(frontCameraPose);
        }
        if (backLeftCameraPose != null) {
            field.getObject(VisionConstants.BACK_LEFT_CAMERA).setPose(backLeftCameraPose);
        }
        if (backRightCameraPose != null) {
            field.getObject(VisionConstants.BACK_RIGHT_CAMERA).setPose(backRightCameraPose);
        }

        // Log vision data once per second
        double currentTime = Timer.getFPGATimestamp();
        if (currentTime - lastLogTime >= 1.0) {
            logVisionData();
            lastLogTime = currentTime;
        }

        // TESTING: angle/distance to hub (alliance-aware)
        int hubTagId = Constants.FieldConstants.getHubTagID();
        if (this.getDetectedTagId() == hubTagId) {
            double angle = Math.toDegrees(getAngleToTag(hubTagId));
            BreakerLog.log("Vision/Aim/AngleToHub", String.format("%.2f", angle));
            BreakerLog.log("Vision/Aim/DistanceToHub", String.format("%.2f", this.getDistanceToTag(hubTagId)) + "m");
        }
    }

    /**
     * Robot → Limelight
     * Sends robot IMU orientation data to Limelight for MegaTag2 pose estimation.
     * Only called when USE_MEGATAG2 is true.
     */
    private void sendRobotOrientationToLimelight(String cameraName) {
        try {
            // Get Pigeon IMU data
            var pigeon = drivetrain.getPigeon2();
            // Rotate the yaw by 180 degrees to fix MegaTag2 inversion: https://www.chiefdelphi.com/t/megatag-2-problem/465022
            double yawOffset = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? 0.0 : Math.PI;
            Rotation3d rotation = pigeon.getRotation3d().rotateBy(new Rotation3d(0.0, 0.0, yawOffset));


            // Get angular velocities (degrees per second)
            double yawRate = Math.toDegrees(pigeon.getAngularVelocityZWorld().getValueAsDouble());
            double pitchRate = Math.toDegrees(pigeon.getAngularVelocityYWorld().getValueAsDouble());
            double rollRate = Math.toDegrees(pigeon.getAngularVelocityXWorld().getValueAsDouble());

            // Convert rotation to degrees (Limelight expects degrees)
            double yaw = Math.toDegrees(rotation.getZ());
            double pitch = Math.toDegrees(rotation.getY());
            double roll = Math.toDegrees(rotation.getX());

            // Send orientation data to Limelight using LimelightHelpers
            LimelightHelpers.SetRobotOrientation(cameraName, yaw, yawRate, pitch, pitchRate, roll, rollRate);
        } catch (Exception e) {
            // If Pigeon is not available, skip orientation update
            System.out.println("ERROR getting data from IMU: " + e.getMessage());
        }
    }

    /**
     * Limelight → Robot
     * Reads pose data from a Limelight camera and updates the drivetrain's pose
     * estimate.
     */
    private void updatePoseEstimate(String cameraName) {

        // Get pose estimate using LimelightHelpers (handles MegaTag/MegaTag2 and coordinate frames)
        PoseEstimate estimate = VisionConstants.USE_MEGATAG2
            ? LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName)
            : LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraName);
        
        // Make sure we have valid pose data
        if (estimate == null) {
            logRejection(cameraName, "No pose data available");
            return;
        }

        Pose2d visionPose = estimate.pose;
        double timestampSeconds = estimate.timestampSeconds;
        int tagCount = estimate.tagCount;     
        
        if (cameraName.equals(VisionConstants.FRONT_CAMERA)) {
            frontCameraPose = visionPose;
            frontCameraEstimate = estimate;
        } else if (cameraName.equals(VisionConstants.BACK_LEFT_CAMERA)) {
            backLeftCameraPose = visionPose;
            backLeftCameraEstimate = estimate;
        } else if (cameraName.equals(VisionConstants.BACK_RIGHT_CAMERA)) {
            backRightCameraPose = visionPose;
            backRightCameraEstimate = estimate;
        }

        // Only use vision data if we have enough tags
        if (tagCount < VisionConstants.MIN_TAG_COUNT) {
            logRejection(cameraName, String.format("Insufficient tags (got %d, need %d)", tagCount, VisionConstants.MIN_TAG_COUNT));
            return;
        }

        // Reject vision measurements during very fast rotation (angular velocity > 720 deg/s)
        // Vision measurements are unreliable during fast spins
        try {
            var pigeon = drivetrain.getPigeon2();
            double angularVelocityDegPerSec = Math.abs(Math.toDegrees(pigeon.getAngularVelocityZWorld().getValueAsDouble()));
            if (angularVelocityDegPerSec > 3600) {
                logRejection(cameraName, String.format("Angular velocity too high (%.1f deg/s > 3600 deg/s)", angularVelocityDegPerSec));
                return;
            }
        } catch (Exception e) {
            // If Pigeon is not available, skip angular velocity check
            System.out.println("ERROR getting data from IMU: " + e.getMessage());
        }

        // Get current pose estimate
        Pose2d currentPose = drivetrain.getLocalizer().getPose();
        double distanceFromOrigin = currentPose.getTranslation().getDistance(new Translation2d(0, 0));

        // If pose is at origin, use resetPose() instead of addVisionMeasurement()
        // resetPose() works immediately without requiring odometry history,
        // whereas addVisionMeasurement() needs odometry buffer entries to fuse with
        if (distanceFromOrigin < 0.1) {
            drivetrain.getLocalizer().resetPose(visionPose);
            updateStatus(cameraName, String.format("ACCEPTED: Reset pose (origin, %d tags)", tagCount));
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

        // Calculate dynamic standard deviations based on tag count and proximity
        // More tags and closer tags = lower std dev = more trust in vision
        Matrix<N3, N1> dynamicStdDevs = calculateDynamicStdDevs(tagCount, estimate.avgTagDist);
        
        // Add vision measurement to pose estimator
        drivetrain.addVisionMeasurement(visionPose, timestampSeconds, dynamicStdDevs);
        updateStatus(cameraName, String.format("ACCEPTED: Fused (%.2fm diff, %d tags @ %.2fm avg, %.1fms latency)", 
            poseDifference, tagCount, estimate.avgTagDist, estimate.latency));
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
        // Check all cameras for valid pose estimates with tags
        if (frontCameraEstimate != null && frontCameraEstimate.tagCount > 0) {
            return true;
        }
        if (backLeftCameraEstimate != null && backLeftCameraEstimate.tagCount > 0) {
            return true;
        }
        if (backRightCameraEstimate != null && backRightCameraEstimate.tagCount > 0) {
            return true;
        }
        return false;
    }

    public boolean isTagDetected(int targetTagId) {
        // Check all cameras for valid pose estimates with tags
        if (frontCameraEstimate != null && frontCameraEstimate.rawFiducials != null) {
            for (var i : frontCameraEstimate.rawFiducials) {
                if (i.id == targetTagId) {
                    return true;
                }
            }
        }
        if (backLeftCameraEstimate != null && backLeftCameraEstimate.rawFiducials != null) {
            for (var i : backLeftCameraEstimate.rawFiducials) {
                if (i.id == targetTagId) {
                    return true;
                }
            }
        }
        if (backRightCameraEstimate != null && backRightCameraEstimate.rawFiducials != null) {
            for (var i : backRightCameraEstimate.rawFiducials) {
                if (i.id == targetTagId) {
                    return true;
                }
            }
        }
        return false;
    }

    /**
     * Gets the ID of the primary tag detected by any camera.
     * Checks all cameras and returns the first tag found.
     */
    public int getDetectedTagId() {
        // Check front camera first
        if (frontCameraEstimate != null && frontCameraEstimate.rawFiducials != null && frontCameraEstimate.rawFiducials.length > 0) {
            return (int) frontCameraEstimate.rawFiducials[0].id;
        }
        // Check back-left camera
        if (backLeftCameraEstimate != null && backLeftCameraEstimate.rawFiducials != null && backLeftCameraEstimate.rawFiducials.length > 0) {
            return (int) backLeftCameraEstimate.rawFiducials[0].id;
        }
        // Check back-right camera
        if (backRightCameraEstimate != null && backRightCameraEstimate.rawFiducials != null && backRightCameraEstimate.rawFiducials.length > 0) {
            return (int) backRightCameraEstimate.rawFiducials[0].id;
        }
        return -1; // No tag detected
    }

    /**
     * Gets the ID of the nearest AprilTag currently detected (any camera).
     * Uses fused pose and field layout to compute distance; returns -1 if no tags detected.
     */
    public int getNearestDetectedTagId() {
        int nearestTagId = -1;
        double minDistance = Double.POSITIVE_INFINITY;
        if (frontCameraEstimate != null && frontCameraEstimate.rawFiducials != null) {
            for (var f : frontCameraEstimate.rawFiducials) {
                int id = (int) f.id;
                double d = getDistanceToTag(id);
                if (d >= 0 && d < minDistance) {
                    minDistance = d;
                    nearestTagId = id;
                }
            }
        }
        if (backLeftCameraEstimate != null && backLeftCameraEstimate.rawFiducials != null) {
            for (var f : backLeftCameraEstimate.rawFiducials) {
                int id = (int) f.id;
                double d = getDistanceToTag(id);
                if (d >= 0 && d < minDistance) {
                    minDistance = d;
                    nearestTagId = id;
                }
            }
        }
        if (backRightCameraEstimate != null && backRightCameraEstimate.rawFiducials != null) {
            for (var f : backRightCameraEstimate.rawFiducials) {
                int id = (int) f.id;
                double d = getDistanceToTag(id);
                if (d >= 0 && d < minDistance) {
                    minDistance = d;
                    nearestTagId = id;
                }
            }
        }
        return nearestTagId;
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


    /** Returns the vector from the robot to the AprilTag (uses field layout for tag position). */
    public Translation2d getRobotToTagTranslation(int tagId) {
        Translation2d tagPosition = getTagPosition(tagId);
        if (tagPosition == null) {
            return null;
        }
        return drivetrain.getRobotToPointTranslation(tagPosition);
    }

    /**
     * Calculates dynamic standard deviations for vision measurements based on tag count and proximity.
     * More tags and closer tags = lower std dev (more trust). A camera with 1 close tag can be trusted
     * more than a camera with 2 far-away tags.
     *
     * @param tagCount Number of tags in view
     * @param avgTagDist Average distance to tags in meters (from PoseEstimate.avgTagDist)
     */
    private Matrix<N3, N1> calculateDynamicStdDevs(int tagCount, double avgTagDist) {
        // Base standard deviations from constants
        double baseX = VisionConstants.VISION_STD_DEVS.get(0, 0);
        double baseY = VisionConstants.VISION_STD_DEVS.get(1, 0);
        double baseTheta = VisionConstants.VISION_STD_DEVS.get(2, 0);

        double trustScore = getTrustScore(tagCount, avgTagDist);
        double trustMultiplier = 1.0 / (1.0 + trustScore);

        // Apply scaling to X and Y, but keep theta high (trust IMU for rotation)
        double dynamicX = baseX * trustMultiplier;
        double dynamicY = baseY * trustMultiplier;
        double dynamicTheta = baseTheta; // Keep rotation std dev constant (trust IMU)

        return VecBuilder.fill(dynamicX, dynamicY, dynamicTheta);
    }

    /**
     * Returns the combined trust score for a vision measurement.
     * Higher score = more trust. Combines tag count and proximity.
     */
    private double getTrustScore(int tagCount, double avgTagDist) {
        double tagTrust = tagCount * VisionConstants.TAG_COUNT_SCALE_FACTOR;
        double proximityTrust = tagCount > 0 ? VisionConstants.PROXIMITY_SCALE_FACTOR / (1.0 + Math.max(avgTagDist, 0)) : 0;
        return tagTrust + proximityTrust;
    }

    /**
     * Logs vision data to both NetworkTables and System.out once per second.
     */
    private void logVisionData() {

        // Get tag IDs from pose estimates
        int[] frontTags = getTagIdsFromEstimate(frontCameraEstimate);
        int[] backLeftTags = getTagIdsFromEstimate(backLeftCameraEstimate);
        int[] backRightTags = getTagIdsFromEstimate(backRightCameraEstimate);
        String frontTagsStr = formatTagArray(frontTags);
        String backLeftTagsStr = formatTagArray(backLeftTags);
        String backRightTagsStr = formatTagArray(backRightTags);

        // Format poses with 2 decimal places
        String frontPoseStr = formatPose(frontCameraPose);
        String backLeftPoseStr = formatPose(backLeftCameraPose);
        String backRightPoseStr = formatPose(backRightCameraPose);
        String fusedPoseStr = formatPose(fusedPose);

        // Fused pose (odometry + vision): X, Y, yaw
        if (fusedPose != null) {
            BreakerLog.log("Vision/FusedPose/X", fusedPose.getX());
            BreakerLog.log("Vision/FusedPose/Y", fusedPose.getY());
            BreakerLog.log("Vision/FusedPose/YawDeg", fusedPose.getRotation().getDegrees());
        }
        // IMU yaw (Pigeon); IMU does not provide X/Y position
        try {
            double imuYawDeg = drivetrain.getPigeon2().getRotation2d().getDegrees();
            BreakerLog.log("Vision/IMU/YawDeg", imuYawDeg);
        } catch (Exception ignored) {
            // Pigeon not available
        }

        double frontTrustScore = frontCameraEstimate != null
                ? getTrustScore(frontCameraEstimate.tagCount, frontCameraEstimate.avgTagDist)
                : Double.NaN;
        double backLeftTrustScore = backLeftCameraEstimate != null
                ? getTrustScore(backLeftCameraEstimate.tagCount, backLeftCameraEstimate.avgTagDist)
                : Double.NaN;
        double backRightTrustScore = backRightCameraEstimate != null
                ? getTrustScore(backRightCameraEstimate.tagCount, backRightCameraEstimate.avgTagDist)
                : Double.NaN;

        double frontDistToFused = (frontCameraPose != null && fusedPose != null)
                ? frontCameraPose.getTranslation().getDistance(fusedPose.getTranslation())
                : Double.NaN;
        double backLeftDistToFused = (backLeftCameraPose != null && fusedPose != null)
                ? backLeftCameraPose.getTranslation().getDistance(fusedPose.getTranslation())
                : Double.NaN;
        double backRightDistToFused = (backRightCameraPose != null && fusedPose != null)
                ? backRightCameraPose.getTranslation().getDistance(fusedPose.getTranslation())
                : Double.NaN;

        BreakerLog.log("Vision/FrontCamera/Tags", frontTagsStr);
        BreakerLog.log("Vision/FrontCamera/Pose", frontPoseStr);
        BreakerLog.log("Vision/FrontCamera/TrustScore", frontTrustScore);
        BreakerLog.log("Vision/FrontCamera/DistToFusedM", frontDistToFused);
        BreakerLog.log("Vision/FrontCamera/Status", frontCameraStatus);
        BreakerLog.log("Vision/FrontCamera/LastRejection", frontCameraLastRejection);

        BreakerLog.log("Vision/BackLeftCamera/Tags", backLeftTagsStr);
        BreakerLog.log("Vision/BackLeftCamera/Pose", backLeftPoseStr);
        BreakerLog.log("Vision/BackLeftCamera/TrustScore", backLeftTrustScore);
        BreakerLog.log("Vision/BackLeftCamera/DistToFusedM", backLeftDistToFused);
        BreakerLog.log("Vision/BackLeftCamera/Status", backLeftCameraStatus);
        BreakerLog.log("Vision/BackLeftCamera/LastRejection", backLeftCameraLastRejection);

        BreakerLog.log("Vision/BackRightCamera/Tags", backRightTagsStr);
        BreakerLog.log("Vision/BackRightCamera/Pose", backRightPoseStr);
        BreakerLog.log("Vision/BackRightCamera/TrustScore", backRightTrustScore);
        BreakerLog.log("Vision/BackRightCamera/DistToFusedM", backRightDistToFused);
        BreakerLog.log("Vision/BackRightCamera/Status", backRightCameraStatus);
        BreakerLog.log("Vision/BackRightCamera/LastRejection", backRightCameraLastRejection);

        BreakerLog.log("Vision/FusedPose/Pose", fusedPoseStr);

        double imuYawForLog = Double.NaN;
        try {
            imuYawForLog = drivetrain.getPigeon2().getRotation2d().getDegrees();
        } catch (Exception ignored) {
        }
        String frontTrustStr = Double.isNaN(frontTrustScore) ? "—" : String.format("%.3f", frontTrustScore);
        String backLeftTrustStr = Double.isNaN(backLeftTrustScore) ? "—" : String.format("%.3f", backLeftTrustScore);
        String backRightTrustStr = Double.isNaN(backRightTrustScore) ? "—" : String.format("%.3f", backRightTrustScore);
        String frontDistStr = Double.isNaN(frontDistToFused) ? "—" : String.format("%.3fm", frontDistToFused);
        String backLeftDistStr = Double.isNaN(backLeftDistToFused) ? "—" : String.format("%.3fm", backLeftDistToFused);
        String backRightDistStr = Double.isNaN(backRightDistToFused) ? "—" : String.format("%.3fm", backRightDistToFused);
        String logMessage = String.format(
                "------------------------------------------------------\n" +
                "- Front Camera: Tags %s, Pose %s, Trust %s, ΔFused %s\n" +
                "- Back-Left Camera: Tags %s, Pose %s, Trust %s, ΔFused %s\n" +
                "- Back-Right Camera: Tags %s, Pose %s, Trust %s, ΔFused %s\n" +
                "- Fused: Pose %s (X=%.2f Y=%.2f Yaw=%.2f)\n" +
                "- IMU Yaw: %.2f deg",
                frontTagsStr, frontPoseStr, frontTrustStr, frontDistStr,
                backLeftTagsStr, backLeftPoseStr, backLeftTrustStr, backLeftDistStr,
                backRightTagsStr, backRightPoseStr, backRightTrustStr, backRightDistStr,
                fusedPoseStr,
                fusedPose != null ? fusedPose.getX() : Double.NaN,
                fusedPose != null ? fusedPose.getY() : Double.NaN,
                fusedPose != null ? fusedPose.getRotation().getDegrees() : Double.NaN,
                imuYawForLog);
        //System.out.println(logMessage);

        BreakerLog.log("Vision/Log", logMessage);
    }

    /**
     * Extracts tag IDs from a PoseEstimate.
     */
    private int[] getTagIdsFromEstimate(PoseEstimate estimate) {
        if (estimate == null || estimate.rawFiducials == null) {
            return new int[0];
        }
        int[] tagIds = new int[estimate.rawFiducials.length];
        for (int i = 0; i < estimate.rawFiducials.length; i++) {
            tagIds[i] = (int) estimate.rawFiducials[i].id;
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
     * Updates the comprehensive status for a camera (includes accepted and rejected measurements).
     */
    private void updateStatus(String cameraName, String status) {
        if (cameraName.equals(VisionConstants.FRONT_CAMERA)) {
            frontCameraStatus = status;
        } else if (cameraName.equals(VisionConstants.BACK_LEFT_CAMERA)) {
            backLeftCameraStatus = status;
        } else if (cameraName.equals(VisionConstants.BACK_RIGHT_CAMERA)) {
            backRightCameraStatus = status;
        }
    }

    /**
     * Logs measurement rejection to NetworkTables.
     * Stores the rejection reason separately and updates the comprehensive status.
     */
    private void logRejection(String cameraName, String reason) {
        String rejectionMessage = "REJECTED: " + reason;
        
        // Store the rejection reason
        if (cameraName.equals(VisionConstants.FRONT_CAMERA)) {
            frontCameraLastRejection = rejectionMessage;
        } else if (cameraName.equals(VisionConstants.BACK_LEFT_CAMERA)) {
            backLeftCameraLastRejection = rejectionMessage;
        } else if (cameraName.equals(VisionConstants.BACK_RIGHT_CAMERA)) {
            backRightCameraLastRejection = rejectionMessage;
        }
        
        // Update the comprehensive status
        updateStatus(cameraName, rejectionMessage);
    }
}
