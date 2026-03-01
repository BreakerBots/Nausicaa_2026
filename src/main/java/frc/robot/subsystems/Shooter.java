package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.BreakerLib.util.commands.TimedWaitUntilCommand;
import frc.robot.BreakerLib.util.factory.BreakerCANCoderFactory;
import frc.robot.BreakerLib.util.logging.BreakerLog;

public class Shooter extends SubsystemBase {

    private final TalonFX shooterFlywheel1Motor = new TalonFX(Constants.ShooterConstants.SHOOTER_FLYWHEEL_1_MOTOR_ID,
            Constants.GeneralConstants.SUPERSTRUCTURE_CANIVORE_BUS);
    private final TalonFX shooterFlywheel2Motor = new TalonFX(Constants.ShooterConstants.SHOOTER_FLYWHEEL_2_MOTOR_ID,
            Constants.GeneralConstants.SUPERSTRUCTURE_CANIVORE_BUS);
    private final TalonFX shooterFlywheel3Motor = new TalonFX(Constants.ShooterConstants.SHOOTER_FLYWHEEL_3_MOTOR_ID,
            Constants.GeneralConstants.SUPERSTRUCTURE_CANIVORE_BUS);

    private final TalonFX hoodMotor = new TalonFX(Constants.ShooterConstants.HOOD_MOTOR_ID,
            Constants.GeneralConstants.SUPERSTRUCTURE_CANIVORE_BUS);
    private final CANcoder hoodEncoder = BreakerCANCoderFactory.createCANCoder(
            Constants.ShooterConstants.HOOD_ENCODER_ID,
            Constants.GeneralConstants.SUPERSTRUCTURE_CANIVORE_BUS,
            Constants.ShooterConstants.HOOD_ENCODER_DISCONTINUITY,
            Rotations.of(Constants.ShooterConstants.HOOD_ENCODER_OFFSET_ROTATIONS),
            SensorDirectionValue.CounterClockwise_Positive);

    public Shooter() {
        // Flywheels 2 and 3 follow flywheel 1 (same direction)
        TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
        flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        flywheelConfig.CurrentLimits = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Constants.ShooterConstants.FLYWHEEL_STATOR_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(Constants.ShooterConstants.FLYWHEEL_SUPPLY_CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true);
        Slot0Configs slot0 = flywheelConfig.Slot0;
        
        slot0.kS = Constants.ShooterConstants.SHOOTER_kS;
        slot0.kV = Constants.ShooterConstants.SHOOTER_kV;
        //slot0.kA = Constants.ShooterConstants.SHOOTER_kA;
        slot0.kP = Constants.ShooterConstants.SHOOTER_kP;
        slot0.kI = Constants.ShooterConstants.SHOOTER_kI;
        slot0.kD = Constants.ShooterConstants.SHOOTER_kD;

         shooterFlywheel1Motor.getConfigurator().apply(flywheelConfig);
         shooterFlywheel2Motor.getConfigurator().apply(flywheelConfig);
         shooterFlywheel3Motor.getConfigurator().apply(flywheelConfig);

        int leaderId = Constants.ShooterConstants.SHOOTER_FLYWHEEL_1_MOTOR_ID;
        shooterFlywheel2Motor.setControl(new Follower(leaderId, MotorAlignmentValue.Aligned));
        shooterFlywheel3Motor.setControl(new Follower(leaderId, MotorAlignmentValue.Aligned));

        TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
        hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        hoodConfig.CurrentLimits = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Constants.ShooterConstants.HOOD_STATOR_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true);
        hoodMotor.getConfigurator().apply(hoodConfig);
    }


    public State state = State.INACTIVE;

    /** Shooter states: flywheel speeds (inactive vs shooting). */
    public enum State {
        
        INACTIVE(Constants.ShooterConstants.SPEED_IDLE),
        SPINNING_UP(Constants.ShooterConstants.SPEED_FLYWHEEL_ACTIVE),
        SHOOTING(Constants.ShooterConstants.SPEED_FLYWHEEL_ACTIVE);



        private double flywheelSpeed;

        private State(double flywheelSpeed) {
             this.flywheelSpeed = flywheelSpeed;
        }

        public double getFlywheelSpeed() {
          return flywheelSpeed;
        }
    }

    public void setState(State newState) {
        State previousState = state;
        state = newState;
        setFlywheelSpeed(state.getFlywheelSpeed());
        // Flywheels 2 and 3 follow flywheel 1 via Follower control in constructor

        BreakerLog.log("Shooter/State/Previous", previousState.toString());
        BreakerLog.log("Shooter/State/Current", state.toString());
        BreakerLog.log("Shooter/State/TargetFlywheelSpeed", state.getFlywheelSpeed());
    }

    public Command setStateCommand(State newState) {
        return Commands.runOnce(() -> setState(newState), this);
    }

    // public boolean isAtTargetVelocity() {
    //     double current = shooterFlywheel1Motor.getVelocity().getValueAsDouble();
    //     double target = state.getFlywheelSpeed();
    //     return Math.abs(current - target) < Constants.ShooterConstants.VELOCITY_TOLERANCE;
    // }

    // --------------- Hood (external encoder, run until target rotations) ---------------

    /** Current hood encoder position in rotations (cumulative). */
    public double getHoodEncoderRotations() {
        return hoodEncoder.getPosition().getValueAsDouble();
    }

    public void runHoodUp() {
        hoodMotor.setControl(new DutyCycleOut(Constants.ShooterConstants.SPEED_HOOD_UP));
    }

    public void runHoodDown() {
        hoodMotor.setControl(new DutyCycleOut(Constants.ShooterConstants.SPEED_HOOD_DOWN));
    }

    public void autoRunHoodDown() {
        hoodMotor.setControl(new DutyCycleOut(Constants.ShooterConstants.AUTO_SPEED_HOOD_DOWN));
    }

    /** Stop the hood motor. */
    public void stopHood() {
        hoodMotor.setControl(new DutyCycleOut(0.0));
    }

    /**
     * Command: move hood toward physical bottom until stall detected, then zero encoder to POSITION_HOOD_SETPOINT_HOME.
     * Use when encoder position is unknown (e.g. after power cycle) to establish the DOWN reference.
     */
    public Command homeHood() {
        return Commands.sequence(
            Commands.runOnce(() -> BreakerLog.log("Shooter/Hood/Homing", "Starting"), this),
            Commands.run(() -> hoodMotor.setControl(
                new VoltageOut(Constants.ShooterConstants.HOOD_HOMING_VOLTAGE)), this)
                .raceWith(new TimedWaitUntilCommand(this::detectHoodHome,
                    Constants.ShooterConstants.HOOD_HOMING_STALL_TIME_SECONDS)
                    .raceWith(Commands.waitSeconds(Constants.ShooterConstants.HOOD_HOMING_TIMEOUT_SECONDS))),
            Commands.runOnce(() -> {
                hoodMotor.setControl(new VoltageOut(0.0));
            }, this),
            Commands.waitSeconds(0.2),
            Commands.runOnce(() -> {
                hoodEncoder.setPosition(Constants.ShooterConstants.POSITION_HOOD_SETPOINT_HOME);
                BreakerLog.log("Shooter/Hood/Homing", "CANcoder zeroed to POSITION_HOOD_SETPOINT_HOME");
            }, this))
            .finallyDo((interrupted) -> BreakerLog.log("Shooter/Hood/Homing", "Finished (interrupted=" + interrupted + ")"));
    }

    /** Detect stall at hood physical bottom. */
    private boolean detectHoodHome() {
        return Math.abs(hoodMotor.getSupplyCurrent().getValueAsDouble())
                >= Constants.ShooterConstants.HOOD_HOMING_DETECT_CURRENT_THRESHOLD;
    }

    /**
     * Command: run hood until encoder reaches targetRotations, then stop.
     * Runs up if target is greater than current position, down if less.
     */
    public Command hoodToRotationsCommand(double targetRotations) {
        double current = getHoodEncoderRotations();
        if (targetRotations > current) {
            return Commands.run(this::runHoodUp, this)
                    .until(() -> getHoodEncoderRotations() >= targetRotations)
                    .andThen(Commands.runOnce(this::stopHood, this));
        } else if (targetRotations < current) {
            return Commands.run(this::runHoodDown, this)
                    .until(() -> getHoodEncoderRotations() <= targetRotations)
                    .andThen(Commands.runOnce(this::stopHood, this));
        } else {
            return Commands.runOnce(this::stopHood, this);
        }
    }

    

    @Override
    public void periodic() {
        logStatus();
    }

    /** One compact line: state, flywheels vel, hood pos/vel. */
    private void logStatus() {
        double v1 = shooterFlywheel1Motor.getVelocity().getValueAsDouble();
        double v2 = shooterFlywheel2Motor.getVelocity().getValueAsDouble();
        double v3 = shooterFlywheel3Motor.getVelocity().getValueAsDouble();
        double hoodPos = getHoodEncoderRotations();
        double hoodVel = hoodMotor.getVelocity().getValueAsDouble();
        String line = String.format("state=%s f1=%.1f f2=%.1f f3=%.1fvel hood=%.2frot %.1fvel",
                state, v1, v2, v3, hoodPos, hoodVel);
        BreakerLog.log("Shooter/Status", line);
        BreakerLog.log("Shooter/HoodPosition", hoodPos);
        BreakerLog.log("Shooter/Flywheel1Speed", v1);
        BreakerLog.log("Shooter/Flywheel2Speed", v2);
        BreakerLog.log("Shooter/Flywheel3Speed", v3);
        BreakerLog.log("Electrical/Shooter/flywheel1", shooterFlywheel1Motor);
        BreakerLog.log("Electrical/Shooter/flywheel2", shooterFlywheel2Motor);
        BreakerLog.log("Electrical/Shooter/flywheel3", shooterFlywheel3Motor);
        BreakerLog.log("Electrical/Shooter/hood", hoodMotor);
    }

    private void setFlywheelSpeed(double speed) {
        if (speed == 0) {
            // This makes sure the flywheel is coasting to a stop, not braking to a stop
            shooterFlywheel1Motor.setControl(new DutyCycleOut(0).withOverrideBrakeDurNeutral(false));
        } else {
            shooterFlywheel1Motor.setControl(new VelocityVoltage(speed).withAcceleration(15));
        }
        
    }

}
