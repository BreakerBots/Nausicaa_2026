package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.BreakerLib.util.logging.BreakerLog;

/**
 * Climb subsystem: single motor driving a chain. Uses an external encoder to track
 * rotations; motor runs until encoder reaches target (UP or DOWN), then stops.
 */
public class Climb extends SubsystemBase {

    private final TalonFX climbMotor = new TalonFX(Constants.ClimbConstants.CLIMB_MOTOR_ID,
            Constants.GeneralConstants.SUPERSTRUCTURE_CANIVORE_BUS);
    private final CANcoder climbEncoder = new CANcoder(Constants.ClimbConstants.CLIMB_ENCODER_ID,
            Constants.GeneralConstants.SUPERSTRUCTURE_CANIVORE_BUS);

    public State state = State.INACTIVE;

    /** Climb states: INACTIVE = stopped; ASCENDING/DESCENDING = run until encoder reaches target. */
    public enum State {
        INACTIVE,
        ASCENDING,
        DESCENDING;
    }

    public Climb() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        climbMotor.getConfigurator().apply(config);
    }

    /** Current encoder position in rotations (cumulative). */
    public double getEncoderRotations() {
        return climbEncoder.getPosition().getValueAsDouble();
    }

    /** Zero the encoder (call when climb is at DOWN and you want DOWN = 0). */
    public void zeroEncoder() {
        climbEncoder.setPosition(0.0);
    }

    public boolean atUpPosition() {
        return getEncoderRotations() >= Constants.ClimbConstants.ROTATIONS_UP;
    }

    public boolean atDownPosition() {
        return getEncoderRotations() <= Constants.ClimbConstants.ROTATIONS_DOWN;
    }

    private void runClimbUp() {
        climbMotor.setControl(new DutyCycleOut(Constants.ClimbConstants.SPEED_ASCENDING));
    }

    private void runClimbDown() {
        climbMotor.setControl(new DutyCycleOut(Constants.ClimbConstants.SPEED_DESCENDING));
    }

    /** Stop the climb motor. */
    public void stop() {
        climbMotor.setControl(new DutyCycleOut(0.0));
    }

    /** Command: run motor toward UP until encoder reaches ROTATIONS_UP, then stop. */
    public Command climbToUpCommand() {
        // return Commands.run(this::runClimbUp, this)
        //         .until(this::atUpPosition)
        //         .andThen(Commands.runOnce(this::stop, this))
        //         .andThen(Commands.runOnce(() -> setState(State.INACTIVE), this));
        return Commands.run(this::runClimbUp, this).finallyDo(this::stop);
    }

    /** Command: run motor toward DOWN until encoder reaches ROTATIONS_DOWN, then stop. */
    public Command climbToDownCommand() {
        // return Commands.run(this::runClimbDown, this)
        //         .until(this::atDownPosition)
        //         .andThen(Commands.runOnce(this::stop, this))
        //         .andThen(Commands.runOnce(() -> setState(State.INACTIVE), this));
        return Commands.run(this::runClimbDown, this).finallyDo(this::stop);

    }

    public void setState(State newState) {
        State previousState = state;
        state = newState;
        if (state == State.INACTIVE) {
            stop();
        }

        BreakerLog.log("Climb/State/Previous", previousState.toString());
        BreakerLog.log("Climb/State/Current", state.toString());
        if (state == State.ASCENDING) {
            BreakerLog.log("Climb/State/TargetRotations", Constants.ClimbConstants.ROTATIONS_UP);
        } else if (state == State.DESCENDING) {
            BreakerLog.log("Climb/State/TargetRotations", Constants.ClimbConstants.ROTATIONS_DOWN);
        }
    }

    /**
     * Returns a command that runs climb to the given state. ASCENDING/DESCENDING run until
     * encoder target then stop; INACTIVE stops immediately.
     */
    public Command setStateCommand(State newState) {
        if (newState == State.INACTIVE) {
            return Commands.runOnce(() -> setState(State.INACTIVE), this);
        }
        if (newState == State.ASCENDING) {
            return climbToUpCommand();
        }
        return climbToDownCommand();
    }

    @Override
    public void periodic() {
        logStatus();
    }

    /** One compact line: state, encoder pos, motor vel/current. */
    private void logStatus() {
        double pos = getEncoderRotations();
        double vel = climbMotor.getVelocity().getValueAsDouble();
        double cur = climbMotor.getStatorCurrent().getValueAsDouble();
        String line = String.format("state=%s pos=%.2frot %.1fvel %.1fA", state, pos, vel, cur);
        BreakerLog.log("Climb/Status", line);
    }

    public void setSpeed(double speed) {
      climbMotor.setControl(new DutyCycleOut(speed));
    }

    public Command setSpeedCommand(double speed) {
      return Commands.runOnce(() -> setSpeed(speed));
    }

}
