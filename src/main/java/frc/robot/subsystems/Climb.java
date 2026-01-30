package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;

public class Climb {
    private final TalonFX climbMotor = new TalonFX(Constants.ClimbConstants.CLIMB_MOTOR_ID,
            Constants.GeneralConstants.DRIVE_CANIVORE_BUS);

    public State state = State.DOWN;

    /**
     * Right now, these states equate to arbitrary positions.
     * Ideally, they'd be based on actual modes for the arm (ie. intake, score,
     * stow, etc.)
     */
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
        
        // Log state change
        // System.out.println("Arm state changed from " + previousState.toString() + " to " + state.toString());
        // BreakerLog.log("Arm/State/Previous", previousState.toString());
        // BreakerLog.log("Arm/State/Current", state.toString());
        // BreakerLog.log("Arm/State/Position", state.getRotation2d().getRotations());
    }

    public Command setStateCommand(State newState) {
        return Commands.runOnce(() -> setState(newState));
    }

    private void setClimbPosition(double position) {
        climbMotor.setControl(new PositionDutyCycle(position));
    }

}
