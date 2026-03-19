package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.BreakerLib.util.factory.BreakerCANCoderFactory;
import frc.robot.BreakerLib.util.logging.BreakerLog;

public class Hood extends SubsystemBase {

    private final TalonFX hoodMotor = new TalonFX(Constants.ShooterConstants.HOOD_MOTOR_ID,
            Constants.GeneralConstants.SUPERSTRUCTURE_CANIVORE_BUS);
    private final CANcoder hoodEncoder = BreakerCANCoderFactory.createCANCoder(
            Constants.ShooterConstants.HOOD_ENCODER_ID,
            Constants.GeneralConstants.SUPERSTRUCTURE_CANIVORE_BUS,
            Constants.ShooterConstants.HOOD_ENCODER_DISCONTINUITY,
            Rotations.of(Constants.ShooterConstants.HOOD_ENCODER_OFFSET_ROTATIONS),
            SensorDirectionValue.CounterClockwise_Positive);

    public Hood() {
        TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
        hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        hoodConfig.CurrentLimits = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Constants.ShooterConstants.HOOD_STATOR_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(Constants.ShooterConstants.HOOD_SUPPLY_CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true);
        hoodMotor.getConfigurator().apply(hoodConfig);
    }

    /** Current hood encoder position in rotations (cumulative). */
    public double getHoodEncoderRotations() {
        return hoodEncoder.getPosition().getValueAsDouble();
    }

    public void runHoodUp() {
        if (getHoodEncoderRotations() >= Constants.ShooterConstants.POSITION_HOOD_MAX) {
            stopHood();
        } else {
            hoodMotor.setControl(new DutyCycleOut(Constants.ShooterConstants.SPEED_HOOD_UP));
        }
    }

    public void runHoodDown() {
        if (getHoodEncoderRotations() <= Constants.ShooterConstants.POSITION_HOOD_MIN) {
            stopHood();
        } else {
            hoodMotor.setControl(new DutyCycleOut(Constants.ShooterConstants.SPEED_HOOD_DOWN));
        }
    }

    public void autoRunHoodDown() {
        hoodMotor.setControl(new DutyCycleOut(Constants.ShooterConstants.AUTO_SPEED_HOOD_DOWN));
    }

    /** Stop the hood motor. */
    public void stopHood() {
        hoodMotor.setControl(new DutyCycleOut(0.0));
    }

    /**
     * Positions hood based on distance to target (meters). Continually updates while running.
     */
    public Command positionHoodForTargetCommand(double distanceMeters) {
        return positionHoodForTargetCommand(() -> distanceMeters);
    }

    /**
     * Positions hood based on distance to target. Distance is re-evaluated each cycle (e.g. for dynamic targets).
     */
    public Command positionHoodForTargetCommand(DoubleSupplier distanceSupplier) {
        final double tolerance = Constants.ShooterConstants.HOOD_TRACKING_TOLERANCE_ROTATIONS;
        return Commands.run(() -> {
            double distance = distanceSupplier.getAsDouble();
            double hoodTarget = TrajectoryManager.getHoodPositionForDistance(distance);
            double current = getHoodEncoderRotations();
            if (hoodTarget > current + tolerance) {
                runHoodUp();
            } else if (hoodTarget < current - tolerance) {
                runHoodDown();
            } else {
                stopHood();
            }
        }, this).finallyDo(this::stopHood);
    }

    public Command hoodToRotationsCommand(double targetRotations) {
        double clamped = MathUtil.clamp(targetRotations,
                Constants.ShooterConstants.POSITION_HOOD_MIN,
                Constants.ShooterConstants.POSITION_HOOD_MAX);
        double tolerance = Constants.ShooterConstants.HOOD_TRACKING_TOLERANCE_ROTATIONS;
        double kP = Constants.ShooterConstants.HOOD_kP;
        return Commands.run(() -> {
            double error = clamped - getHoodEncoderRotations();
            double output = MathUtil.clamp(-kP * error, -1.0, 1.0);
            hoodMotor.setControl(new DutyCycleOut(output));
        }, this)
                .until(() -> Math.abs(getHoodEncoderRotations() - clamped) <= tolerance)
                .finallyDo(this::stopHood);
    }

    @Override
    public void periodic() {
        
        double hoodPos = getHoodEncoderRotations();
        double hoodVel = hoodMotor.getVelocity().getValueAsDouble();
        BreakerLog.log("Hood/PositionRot", hoodPos, true);
        BreakerLog.log("Hood/VelocityRps", hoodVel, true);
        if (BreakerLog.isVerboseLogging()) {
            BreakerLog.log("Electrical/Hood/motor", hoodMotor);
        }
    }
}
