package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import java.util.Optional;

/**
 * Vision subsystem for handling two LimeLight4 (LL4)cameras for localization.
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
      // Send robot orientation to Limelights for MegaTag2
      sendRobotOrientationToLimelight(frontCameraData);
      sendRobotOrientationToLimelight(backCameraData);
      
      // Process vision poses from both cameras
      updatePoseEstimate(frontCameraData, "front_camera");
      updatePoseEstimate(backCameraData, "back_camera");
  
      // Update Field2d with all poses (fused, and one for each camera) for Elastic dashboard visualization
      fusedPose = drivetrain.getLocalizer().getPose();
      field.getRobotObject().setPose(fusedPose); // Fused pose (odometry + vision)
      
      // Add camera poses to field visualization
      if (frontCameraPose != null) {
          field.getObject("front_camera").setPose(frontCameraPose);
      }
      if (backCameraPose != null) {
          field.getObject("back_camera").setPose(backCameraPose);
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
        double[] robotOrientation = {yaw, pitch, roll, yawRate, pitchRate, rollRate};
        cameraData.getEntry("robot_orientation_set").setDoubleArray(robotOrientation);
    } catch (Exception e) {
        // If Pigeon is not available, skip orientation update
    }
  }

  /**
   * Limelight → Robot
   * Reads pose data from a Limelight camera and updates the drivetrain's pose estimate.
   */
  private void updatePoseEstimate(NetworkTable cameraData, String cameraName) {
    // Determine which pose entry to use based on alliance
    String poseEntryName;
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
        poseEntryName = "botpose_orb_wpiblue";
    } else {
        poseEntryName = "botpose_orb_wpired";
    }

    // Get pose data array from Limelight
    // Format: [x, y, z, roll, pitch, yaw, latency, tagCount]
    double[] botpose = cameraData.getEntry(poseEntryName).getDoubleArray(new double[0]);
    
    // Check if we have valid pose data
    if (botpose.length < 8) {
        return; // Invalid data
    }

    double x = botpose[0];
    double y = botpose[1];
    double yaw = botpose[5];
    double latency = botpose[6]; // milliseconds
    double tagCount = botpose[7];

    // Create pose from vision data (using x, y, and yaw)
    Pose2d visionPose = new Pose2d(x, y, Rotation2d.fromDegrees(yaw));
    
    // Store vision pose for visualization
    if (cameraName.equals("front_camera")) {
        frontCameraPose = visionPose;
    } else {
        backCameraPose = visionPose;
    }

    // Only use vision data if we have enough tags
    if (tagCount < VisionConstants.MIN_TAG_COUNT) {
        return;
    }

    // Get current pose estimate to check if vision measurement is reasonable
    Pose2d currentPose = drivetrain.getLocalizer().getPose();
    double poseDifference = visionPose.getTranslation().getDistance(currentPose.getTranslation());
    
    // Only add vision measurement if it's within reasonable distance of current estimate
    //if (poseDifference > VisionConstants.MAX_POSE_DIFFERENCE) {
    //    return; // Vision measurement seems unreliable
    //}

    // Calculate timestamp accounting for latency
    // Latency is in milliseconds, convert to seconds
    double timestampSeconds = Timer.getFPGATimestamp() - (latency / 1000.0);

    // Add vision measurement to pose estimator
    drivetrain.addVisionMeasurement(visionPose, timestampSeconds, VisionConstants.VISION_STD_DEVS);
  }

  /**
   * Calculates the angle error (in radians) to face a target position on the field.
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
      return (int)frontTid[0];
    }
    double[] backTid = backCameraData.getEntry("tid").getDoubleArray(new double[0]);
    if (backTid.length > 0) {
      return (int)backTid[0];
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
}
