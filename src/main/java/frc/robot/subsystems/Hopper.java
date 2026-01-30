package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.BreakerLib.util.logging.BreakerLog;

public class Hopper extends SubsystemBase {
    
    private final TalonFX hopperRollerMotor = new TalonFX(Constants.HopperConstants.HOPPER_MOTOR_ID,
            Constants.GeneralConstants.DRIVE_CANIVORE_BUS);

    public State state = State.INACTIVE;

    /** Hopper states: roller speed (inactive vs feeding). */
    public enum State {
 
        INACTIVE(Constants.HopperConstants.SPEED_INACTIVE),
        FEEDING(Constants.HopperConstants.SPEED_FEEDING);

        private double rollerSpeed;

        private State(double rollerSpeed) {
             this.rollerSpeed = rollerSpeed;
        }

        public double getRollerSpeed() {
          return rollerSpeed;
      }

    }
    
    public void setState(State newState) {
        State previousState = state;
        state = newState;
        setRollerSpeed(state.getRollerSpeed());

        BreakerLog.log("Hopper/State/Previous", previousState.toString());
        BreakerLog.log("Hopper/State/Current", state.toString());
        BreakerLog.log("Hopper/State/RollerSpeed", state.getRollerSpeed());
    }

    public Command setStateCommand(State newState) {
        return Commands.runOnce(() -> setState(newState), this);
    }


    @Override
    public void periodic() {
        logStatus();
    }

    
    /** One compact line: state, roller cmd/vel/current. */
    private void logStatus() {
        double vel = hopperRollerMotor.getVelocity().getValueAsDouble();
        double cur = hopperRollerMotor.getStatorCurrent().getValueAsDouble();
        String line = String.format("state=%s roller=%.2fcmd %.1fvel %.1fA",
                state, state.getRollerSpeed(), vel, cur);
        BreakerLog.log("Hopper/Status", line);
    }

    private void setRollerSpeed(double speed) {
        hopperRollerMotor.setControl(new DutyCycleOut(speed));
    }

}
