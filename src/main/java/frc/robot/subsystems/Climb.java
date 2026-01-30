package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.BreakerLib.util.logging.BreakerLog;

public class Climb extends SubsystemBase {

    private final TalonFX climbMotor = new TalonFX(Constants.ClimbConstants.CLIMB_MOTOR_ID,
            Constants.GeneralConstants.DRIVE_CANIVORE_BUS);

    public State state = State.DOWN;

    /** Climb states: target encoder rotations (UP vs DOWN). */
    public enum State {
 
        UP(Constants.ClimbConstants.ROTATIONS_UP),
        DOWN(Constants.ClimbConstants.ROTATIONS_DOWN);

        private double rotations;

        private State(double rotations) {
             this.rotations = rotations;
        }

        public double getClimbPosition() {
            return rotations;
        }

    }
    
    public void setState(State newState) {
        State previousState = state;
        state = newState;
        setClimbPosition(state.getClimbPosition());

        BreakerLog.log("Climb/State/Previous", previousState.toString());
        BreakerLog.log("Climb/State/Current", state.toString());
        BreakerLog.log("Climb/State/TargetRotations", state.getClimbPosition());
    }

    public Command setStateCommand(State newState) {
        return Commands.runOnce(() -> setState(newState), this);
    }


    @Override
    public void periodic() {
        logStatus();
    }


    /** One compact line: state, motor pos/vel/current. */
    private void logStatus() {
        double pos = climbMotor.getPosition().getValueAsDouble();
        double vel = climbMotor.getVelocity().getValueAsDouble();
        double cur = climbMotor.getStatorCurrent().getValueAsDouble();
        String line = String.format("state=%s pos=%.2frot %.1fvel %.1fA",
                state, pos, vel, cur);
        BreakerLog.log("Climb/Status", line);
    }

    private void setClimbPosition(double position) {
        climbMotor.setControl(new PositionDutyCycle(position));
    }
}
