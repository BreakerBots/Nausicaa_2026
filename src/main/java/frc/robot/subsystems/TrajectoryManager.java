package frc.robot.subsystems;

import java.util.Collections;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.BreakerLib.physics.BreakerVector2;
import frc.robot.BreakerLib.util.math.interpolation.maps.BreakerInterpolatingTreeMap;

/**
 * Manages trajectory calculations for shooting: distance to target, hood angle from pose or distance.
 */
public class TrajectoryManager extends SubsystemBase {

    /**
     * Distance, hood position, and flywheel speed. Tune through testing – add/remove/adjust as needed.
     * Hood position must stay between POSITION_HOOD_MIN and POSITION_HOOD_MAX.
     */
    private record ShootEntry(double distanceM, double hoodRot, double flywheelSpeed) {}


    // for MHS only: add 33.23401872 inches to each measurement (corner to center of hub)
    private static final ShootEntry[] SHOOT_LOOKUP_TABLE = {
        new ShootEntry(1.47, 0.03, 50),  // MHS
        new ShootEntry(1.5902686, 0.04, 52),       
        new ShootEntry(1.8950686, 0.07, 54),       
        new ShootEntry(2.1998686, 0.1, 56),      
        new ShootEntry(3.1143686, 0.19, 60),  
        new ShootEntry(4.0286686, 0.26, 63),  
        new ShootEntry(4.9430686, 0.3, 66),   
    };

    private static final BreakerInterpolatingTreeMap<Double, BreakerVector2> shootLookup = buildShootLookup();

    private final Drivetrain drivetrain;

    public TrajectoryManager(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    private static BreakerInterpolatingTreeMap<Double, BreakerVector2> buildShootLookup() {
        BreakerInterpolatingTreeMap<Double, BreakerVector2> map = new BreakerInterpolatingTreeMap<>();
        for (ShootEntry e : SHOOT_LOOKUP_TABLE) {
            map.put(e.distanceM(), new BreakerVector2(e.hoodRot(), e.flywheelSpeed()));
        }
        return map;
    }

    /** Returns the distance in meters from the robot to the given field point. */
    public double getDistanceToPoint(Translation2d targetPoint) {
        return drivetrain.getRobotToPointTranslation(targetPoint).getNorm();
    }

    /** Returns the distance in meters from the robot to the current shooting target (hub for pose). */
    public double getDistanceToTarget() {
        return getDistanceToPoint(Constants.FieldConstants.getTargetForPose(drivetrain.getLocalizer().getPose()));
    }

    /**
     * Returns hood position in encoder rotations for the given distance to target in meters.
     * Uses linear interpolation through SHOOT_LOOKUP_TABLE. Extrapolates only for
     * distances beyond the table max (long shots); below-min distances use the table min.
     * Result is clamped to mechanical limits.
     */
    public static double getHoodPositionForDistance(double distanceToTargetMeters) {
        BreakerVector2 vals = getShootValuesForDistance(distanceToTargetMeters);
        if (vals == null) return Constants.ShooterConstants.POSITION_HOOD_MIN;
        return MathUtil.clamp(vals.getX(), Constants.ShooterConstants.POSITION_HOOD_MIN, Constants.ShooterConstants.POSITION_HOOD_MAX);
    }

    /**
     * Returns flywheel speed (rotations/sec) for the given distance to target in meters.
     */
    public static double getFlywheelSpeedForDistance(double distanceToTargetMeters) {
        BreakerVector2 shootValues = getShootValuesForDistance(distanceToTargetMeters);
        if (shootValues != null) {
            return shootValues.getY();
        } else {
            return Constants.ShooterConstants.SPEED_FLYWHEEL_ACTIVE;
        }
    }

    /** Returns flywheel speed for current distance to target (via getDistanceToTarget). Falls back to constant if distance invalid. */
    public double getFlywheelSpeedForDistance() {
        double distance = getDistanceToTarget();
        // If the distance is bogus, just use our default speed
        if (Double.isNaN(distance) || 
            distance < Constants.ShooterConstants.SHOOTER_RANGE_MIN || 
            distance > Constants.ShooterConstants.SHOOTER_RANGE_MAX) {
            return Constants.ShooterConstants.SPEED_FLYWHEEL_ACTIVE;
        } else {
            return getFlywheelSpeedForDistance(distance);
        }
    }

    /** Returns (hood, flywheel) for the given distance; null if lookup empty. */
    private static BreakerVector2 getShootValuesForDistance(double distanceToTargetMeters) {
        if (shootLookup.isEmpty()) return null;
        double dMax = Collections.max(shootLookup.keySet());
        int tableLen = SHOOT_LOOKUP_TABLE.length;

        BreakerVector2 result;
        if (distanceToTargetMeters >= dMax && tableLen >= 2) {
            ShootEntry p0 = SHOOT_LOOKUP_TABLE[tableLen - 2];
            ShootEntry p1 = SHOOT_LOOKUP_TABLE[tableLen - 1];
            double denom = p1.distanceM() - p0.distanceM();
            double t = denom != 0 ? (distanceToTargetMeters - p0.distanceM()) / denom : 1;
            // Use linear extrapolation: start + t*(end - start). Do NOT use interpolate() -
            // MathUtil.interpolate clamps t to [0,1], which would cap us at the last table entry.
            double hood = p0.hoodRot() + t * (p1.hoodRot() - p0.hoodRot());
            double flywheel = p0.flywheelSpeed() + t * (p1.flywheelSpeed() - p0.flywheelSpeed());
            result = new BreakerVector2(hood, flywheel);
        } else {
            result = shootLookup.getInterpolatedValue(distanceToTargetMeters);
        }
        return result;
    }
    
   /**
    * True if hood encoder is within tolerance of the
    * trajectory hood setpoint for this distance (after clamping target to mechanical limits).
    */
   public static boolean isHoodPositionGoodToShoot(double currentHoodRotations, double distanceToTargetMeters) {
       double target = getHoodPositionForDistance(distanceToTargetMeters);
       double clamped = MathUtil.clamp(target,
               Constants.ShooterConstants.POSITION_HOOD_MIN,
               Constants.ShooterConstants.POSITION_HOOD_MAX);
       return Math.abs(currentHoodRotations - clamped)
               <= Constants.ShooterConstants.HOOD_GTS_TOLERANCE_ROTATIONS;
   }


   // We should change this to use BreakerLib 
   private static double headingTolerancePercentForDistance(double dMeters) {
       if (dMeters <= 1.0) {
           return 5.0;
       }
       if (dMeters <= 1.5) {
           return MathUtil.interpolate(5.0, 4.0, (dMeters - 1.0) / 0.5);
       }
       if (dMeters <= 2.0) {
           return MathUtil.interpolate(4.0, 3.5, (dMeters - 1.5) / 0.5);
       }
       if (dMeters <= 2.5) {
           return MathUtil.interpolate(3.5, 3.0, (dMeters - 2.0) / 0.5);
       }
       if (dMeters <= 3.0) {
           return MathUtil.interpolate(3.0, 2.5, (dMeters - 2.5) / 0.5);
       }
       if (dMeters <= 4.0) {
           return MathUtil.interpolate(2.5, 2.0, (dMeters - 3.0) / 1.0);
       }
       return 2.0;
   }


   /**
    * True when {@code |angleToTarget|} is within a distance-dependent cap. Tolerance is {@code (percent/100)·π} rad
    * where {@code percent} is interpolated from the table (5% at ≤1 m down to 2% at ≥4 m).
    */
   public static boolean isHeadingGoodToShoot(double absAngleErrorRad, double distanceToTargetMeters) {
       double p = headingTolerancePercentForDistance(distanceToTargetMeters);
       double maxErrRad = (p / 100.0) * Math.PI;
       return absAngleErrorRad <= maxErrRad;
   }

}
