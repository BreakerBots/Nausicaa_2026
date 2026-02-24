package frc.robot.BreakerLib.util.factory;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.measure.Angle;

/**
 * Creates and configures CTRE CANcoders so they report absolute position correctly.
 *
 * It sets:
 * 1. Magnet offset – Shifts the reading so a chosen physical pose (e.g. stowed) maps to the desired value.
 * 2. Discontinuity point – Where the 0→1 rollover happens (e.g. 0.5 for ±180°), so the wrap is outside the mechanism's range.
 * 3. Sensor direction – Which rotation direction is positive.
 *
 * With this, getPosition() gives the right value on power-up without homing or 
 * zeroing. It's intended for on-axis (1:1) mechanisms where each absolute angle 
 * corresponds to a unique mechanism pose.
 */
public class BreakerCANCoderFactory {

    public static CANcoder createCANCoder(int deviceID, CANBus canBus, double absoluteSensorDiscontinuityPoint, Angle absoluteOffset, SensorDirectionValue encoderDirection) {
        CANcoder encoder = new CANcoder(deviceID, canBus);
        configExistingCANCoder(encoder, absoluteSensorDiscontinuityPoint, absoluteOffset, encoderDirection);
        return encoder;
    }

    public static void configExistingCANCoder(CANcoder encoder, double absoluteSensorDiscontinuityPoint, Angle absoluteOffset, SensorDirectionValue encoderDirection) {
        CANcoderConfiguration config =  new CANcoderConfiguration();
        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = absoluteSensorDiscontinuityPoint;
        config.MagnetSensor.withMagnetOffset(absoluteOffset);
        config.MagnetSensor.SensorDirection = encoderDirection;
        encoder.getConfigurator().apply(config);    
    }

    public static class AbsoluteSensorRange {
        public static final double ZERO_TO_ONE = 1.0;
        public static final double SIGNED_PLUS_MINUS_HALF = 0.5;
    }
}
