package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.BreakerLib.util.commands.TimedWaitUntilCommand;
import frc.robot.BreakerLib.util.logging.BreakerLog;

/**
 * Climb subsystem: setpoint-based control using external encoder.
 * Run home() to establish the DOWN reference.
 */
public class Climb extends SubsystemBase {

    private final TalonFX climbMotor = new TalonFX(Constants.ClimbConstants.CLIMB_MOTOR_ID,
            Constants.GeneralConstants.SUPERSTRUCTURE_CANIVORE_BUS);
    private final CANcoder climbEncoder = new CANcoder(Constants.ClimbConstants.CLIMB_ENCODER_ID,
            Constants.GeneralConstants.SUPERSTRUCTURE_CANIVORE_BUS);

    /** Which setpoint are we moving toward? */
    private double targetSetpoint = Constants.ClimbConstants.SETPOINT_DOWN;

    private final PIDController pidController = new PIDController(
        Constants.ClimbConstants.PID_kP,
        Constants.ClimbConstants.PID_kI,
        Constants.ClimbConstants.PID_kD);

    public State state = State.INACTIVE;

    public enum State {
        INACTIVE(0.0),
        EXTENDING(Constants.ClimbConstants.SPEED_EXTENDING),
        ASCENDING(Constants.ClimbConstants.SPEED_ASCENDING),
        DESCENDING(Constants.ClimbConstants.SPEED_DESCENDING),
        RETRACTING(Constants.ClimbConstants.SPEED_RETRACTING),
        HOMING(0.0);

        private final double speed;

        private State(double speed) {
            this.speed = speed;
        }

        public double getSpeed() {
            return speed;
        }
    }

    public Climb() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        climbMotor.getConfigurator().apply(config);
        
        // Configure PID tolerance
        pidController.setTolerance(Constants.ClimbConstants.SETPOINT_TOLERANCE);
    }

    /** Current encoder position in rotations. Uses motor encoder if USE_MOTOR_ENCODER is true, otherwise CANcoder. */
    public double getEncoderRotations() {
        // CANcoder: use external encoder
        return climbEncoder.getPosition().getValueAsDouble();
    }

    /** Zero the encoder to SETPOINT_DOWN (call when climb is at DOWN position). */
    public void zeroEncoder() {
        climbEncoder.setPosition(Constants.ClimbConstants.SETPOINT_DOWN);
        BreakerLog.log("Climb/Encoder", "CANcoder zeroed to SETPOINT_DOWN");
    }

    /**
     * The climb encoder rotates about 2 full turns between DOWN and UP. 
     * Absolute position only gives you 0–1 rotations, so you can’t tell which of the two turns you’re in.
     * Instead, this command moves the climb toward the DOWN limit until we detect a stall,
     * then zeros the encoder to SETPOINT_DOWN and sets the state to INACTIVE.
     */
    public Command home() {
        return Commands.sequence(
            Commands.runOnce(() -> {
            // Set to HOMING so periodic() doesn't overwrite our motor control
            setState(State.HOMING);
                BreakerLog.log("Climb/Homing", "Starting");
            }, this),
            Commands.runOnce(() -> setHomingCurrents(true), this),
            Commands.run(() -> climbMotor.setControl(
                new VoltageOut(Constants.ClimbConstants.HOMING_VOLTAGE)), this)
                    .raceWith(new TimedWaitUntilCommand(this::detectHome,
                        Constants.ClimbConstants.HOMING_STALL_TIME_SECONDS)
                            .raceWith(Commands.waitSeconds(Constants.ClimbConstants.HOMING_TIMEOUT_SECONDS))),
            Commands.runOnce(() -> {
                climbMotor.setControl(new VoltageOut(0.0));
            }, this),
            Commands.waitSeconds(0.2),
            Commands.runOnce(() -> {
                climbEncoder.setPosition(Constants.ClimbConstants.SETPOINT_DOWN);
                BreakerLog.log("Climb/Homing", "CANcoder zeroed to SETPOINT_DOWN");
                setState(State.INACTIVE);
            }, this))
            .finallyDo((interrupted) -> {
                BreakerLog.log("Climb/Homing", "Finished (interrupted=" + interrupted + ")");
                setHomingCurrents(false);
            });
    }

    /** Detect stall at DOWN limit. */
    private boolean detectHome() {
        return Math.abs(climbMotor.getSupplyCurrent().getValueAsDouble())
                >= Constants.ClimbConstants.HOMING_DETECT_CURRENT_THRESHOLD.in(Amps);
    }

    /** Set current limits for homing, which is loosened so the motor can stall without tripping. */
    private void setHomingCurrents(boolean isHoming) {
        climbMotor.getConfigurator().apply(isHoming
                ? Constants.ClimbConstants.HOMING_CURRENT_LIMITS
                : Constants.ClimbConstants.NORMAL_CURRENT_LIMITS);
    }

    /** Check if within tolerance of the given setpoint. */
    public boolean atSetpoint(double setpoint) {
        double error = Math.abs(getPositionError(setpoint));
        return error <= Constants.ClimbConstants.SETPOINT_TOLERANCE;
    }

    /**
     * True when climber is fully retracted (at DOWN) or actively retracting.
     * Use to decide: trigger should run extend.
     */
    public boolean isRetractedOrRetracting() {
        return state == State.RETRACTING
            || (state == State.INACTIVE && atSetpoint(Constants.ClimbConstants.SETPOINT_DOWN));
    }

    /**
     * True when climber is fully extended (at UP) or actively extending.
     * Use to decide: trigger should run retract.
     */
    public boolean isExtendedOrExtending() {
        return state == State.EXTENDING
            || (state == State.INACTIVE && atSetpoint(Constants.ClimbConstants.SETPOINT_UP));
    }

    /** Get position error (current - target). Positive = above target, negative = below target. */
    public double getPositionError(double setpoint) {
        return getEncoderRotations() - setpoint;
    }

    /** Move toward setpoint using PID control with the speed from current state. 
     * This gets a little more complicated because we want to prevent the motor from going backwards if the PID output is in the wrong direction.
     */
    private void moveToSetpoint(double setpoint) {
        targetSetpoint = setpoint;
        
        // Step 1: Get the target speed (% of motor power) for this state (positive = UP, negative = DOWN)
        double stateSpeed = state.getSpeed();
        double stateSpeedMagnitude = Math.abs(stateSpeed);
        
        // Step 2: Calculate PID output (how much correction we need)
        double currentPosition = getEncoderRotations();
        double pidOutput = pidController.calculate(currentPosition, setpoint);
        
        // Step 3: Limit/clamp PID output to not exceed state's max speed magnitude
        double clampedPidOutput = Math.max(-stateSpeedMagnitude, Math.min(stateSpeedMagnitude, pidOutput));
        
        // Step 4: Check if PID is going in the correct direction
        boolean pidGoingUp = clampedPidOutput > 0;
        boolean stateGoingUp = stateSpeed > 0;
        boolean directionsMatch = (pidGoingUp == stateGoingUp);
        
        // Step 5: Use PID if direction is correct, otherwise use state speed (prevents going backwards)
        double motorSpeed;
        if (directionsMatch) {
            motorSpeed = clampedPidOutput;  // PID is correct, use it
        } else {
            motorSpeed = stateSpeed;  // PID is wrong direction, use state speed instead
        }
        
        // Step 6: Apply speed to motor
        climbMotor.setControl(new DutyCycleOut(motorSpeed));
    }

    /** Stop the climb motor. */
    public void stop() {
        climbMotor.setControl(new DutyCycleOut(0.0));
    }

    public void setState(State newState) {
        State previousState = state;
        state = newState;
        
        if (state == State.INACTIVE) {
            stop();
            targetSetpoint = getEncoderRotations(); // Remember current position
        } else if (state == State.RETRACTING || state == State.ASCENDING) {
            targetSetpoint = Constants.ClimbConstants.SETPOINT_DOWN; // Remember current position
        } else if (state == State.EXTENDING || state == State.DESCENDING) {
            targetSetpoint = Constants.ClimbConstants.SETPOINT_UP; // Remember current position
        }

        BreakerLog.log("Climb/StateChange", previousState + " -> " + state);
        BreakerLog.log("Climb/State/Previous", previousState.toString());
        BreakerLog.log("Climb/State/Current", state.toString());
        BreakerLog.log("Climb/TargetSetpoint", targetSetpoint);
        
        if (state != State.INACTIVE) {
            BreakerLog.log("Climb/State/Speed", state.getSpeed());
        }
    }

    /** Command: extend to UP setpoint (fast). */
    public Command extend() {
        return Commands.runOnce(() -> setState(State.EXTENDING), this)
                .andThen(Commands.run(() -> moveToSetpoint(Constants.ClimbConstants.SETPOINT_UP), this)
                        .until(() -> atSetpoint(Constants.ClimbConstants.SETPOINT_UP)))
                .andThen(Commands.runOnce(() -> setState(State.INACTIVE), this));
    }

    /** Command: ascend to DOWN setpoint (slow). */
    public Command ascend() {
        return Commands.runOnce(() -> setState(State.ASCENDING), this)
                .andThen(Commands.run(() -> moveToSetpoint(Constants.ClimbConstants.SETPOINT_CLIMBED), this)
                        .until(() -> atSetpoint(Constants.ClimbConstants.SETPOINT_CLIMBED)))
                .andThen(Commands.runOnce(() -> setState(State.INACTIVE), this));
    }

    /** Command: descend to UP setpoint (slow). */
    public Command descend() {
        return Commands.runOnce(() -> setState(State.DESCENDING), this)
                .andThen(Commands.run(() -> moveToSetpoint(Constants.ClimbConstants.SETPOINT_UP), this)
                        .until(() -> atSetpoint(Constants.ClimbConstants.SETPOINT_UP)))
                .andThen(Commands.runOnce(() -> setState(State.INACTIVE), this));
    }

    /** Command: retract to DOWN setpoint (fast). */
    public Command retract() {
        return Commands.runOnce(() -> setState(State.RETRACTING), this)
                .andThen(Commands.run(() -> moveToSetpoint(Constants.ClimbConstants.SETPOINT_DOWN), this)
                        .until(() -> atSetpoint(Constants.ClimbConstants.SETPOINT_DOWN)))
                .andThen(Commands.runOnce(() -> setState(State.INACTIVE), this));
    }

    /** Command: run motor up while held. Raw duty cycle, no setpoint. */
    public Command runUp() {
        return Commands.run(() -> climbMotor.setControl(new DutyCycleOut(Constants.ClimbConstants.SPEED_EXTENDING)), this)
                .finallyDo(this::stop);
    }

    /** Command: run motor down while held. Raw duty cycle, no setpoint. */
    public Command runDown() {
        return Commands.run(() -> climbMotor.setControl(new DutyCycleOut(Constants.ClimbConstants.SPEED_RETRACTING)), this)
                .finallyDo(this::stop);
    }

    /** Command: hold to extend toward UP; release to stop. For manual adjustment. */
    public Command extendWhileHeld() {
        return Commands.run(() -> {
            setState(State.EXTENDING);
            moveToSetpoint(Constants.ClimbConstants.SETPOINT_UP);
        }, this).finallyDo(() -> setState(State.INACTIVE));
    }

    /** Command: hold to retract toward DOWN; release to stop. For manual adjustment. */
    public Command retractWhileHeld() {
        return Commands.run(() -> {
            setState(State.RETRACTING);
            moveToSetpoint(Constants.ClimbConstants.SETPOINT_DOWN);
        }, this).finallyDo(() -> setState(State.INACTIVE));
    }

    @Override
    public void periodic() {
        // Continue moving toward setpoint if in a moving state
        if (state != State.INACTIVE && state != State.HOMING) {
            moveToSetpoint(targetSetpoint);
            
            // Check if we've reached the setpoint and stop if so
            if (atSetpoint(targetSetpoint)) {
                setState(State.INACTIVE);
            }
        }
        
        double position = getEncoderRotations();
        double velocity = climbMotor.getVelocity().getValueAsDouble();
        if (BreakerLog.isVerboseLogging()) {
            BreakerLog.log("Electrical/Climb/climb", climbMotor);
            BreakerLog.log("Climb/Homing", detectHome());
        }
    }
}
