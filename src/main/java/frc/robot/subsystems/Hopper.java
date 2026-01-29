package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.BreakerLib.util.logging.BreakerLog;

public class Hopper {
    private final TalonFX hopperRollerMotor = new TalonFX(Constants.HopperConstants.HOPPER_MOTOR_ID,
            Constants.GeneralConstants.DRIVE_CANIVORE_BUS);

    public State state = State.INACTIVE;

    /**
     * Right now, these states equate to arbitrary positions.
     * Ideally, they'd be based on actual modes for the arm (ie. intake, score,
     * stow, etc.)
     */
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
        
        // Log state change
        // System.out.println("Arm state changed from " + previousState.toString() + " to " + state.toString());
        // BreakerLog.log("Arm/State/Previous", previousState.toString());
        // BreakerLog.log("Arm/State/Current", state.toString());
        // BreakerLog.log("Arm/State/Position", state.getRotation2d().getRotations());
    }

    public Command setStateCommand(State newState) {
        return Commands.runOnce(() -> setState(newState));
    }

    private void setRollerSpeed(double speed) {
        hopperRollerMotor.setControl(new DutyCycleOut(speed));
    }

}
