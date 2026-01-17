package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.BreakerLib.util.logging.BreakerLog;

public class Intake {
    private final TalonFX extenderMotor = new TalonFX(Constants.IntakeConstants.EXTENDER_MOTOR_ID,
            Constants.GeneralConstants.SUPERSTRUCTURE_CANIVORE_BUS);

    private final TalonFX rollerMotor = new TalonFX(Constants.IntakeConstants.ROLLER_MOTOR_ID,
            Constants.GeneralConstants.SUPERSTRUCTURE_CANIVORE_BUS);
    
    public State state = State.STOWED;

    /**
     * Right now, these states equate to arbitrary positions.
     * Ideally, they'd be based on actual modes for the arm (ie. intake, score,
     * stow, etc.)
     */
    public enum State {
        
        STOWED(Constants.IntakeConstants.POSITION_STOWED, Constants.IntakeConstants.SPEED_IDLE),
        EXTENDED_IDLE(Constants.IntakeConstants.POSITION_EXTENDED, Constants.IntakeConstants.SPEED_IDLE),
        EXTENDED_INTAKING(Constants.IntakeConstants.POSITION_EXTENDED, Constants.IntakeConstants.SPEED_INTAKE),
        EXTENDED_EXTAKING(Constants.IntakeConstants.POSITION_EXTENDED, Constants.IntakeConstants.SPEED_EXTAKE),
        FEED_JIGGLE_HIGH(Constants.IntakeConstants.POSITION_JIGGLE_HIGH, Constants.IntakeConstants.SPEED_INTAKE),
        FEED_JIGGLE_LOW(Constants.IntakeConstants.POSITION_JIGGLE_LOW, Constants.IntakeConstants.SPEED_INTAKE);

        private Rotation2d rotation;
        private double speed;

        private State(Rotation2d rotation, double speed) {
             this.rotation = rotation;
             this.speed = speed;
        }

        public Rotation2d getRotation2d() {
            return rotation;
        }

        public double getSpeed() {
          return speed;
      }

    }

    
    public void setState(State newState) {
        State previousState = state;
        state = newState;
        setIntakePosition(state.getRotation2d().getRotations());
        setRollerSpeed(state.getRotation2d().getRotations());
        
        // Log state change
        // System.out.println("Arm state changed from " + previousState.toString() + " to " + state.toString());
        // BreakerLog.log("Arm/State/Previous", previousState.toString());
        // BreakerLog.log("Arm/State/Current", state.toString());
        // BreakerLog.log("Arm/State/Position", state.getRotation2d().getRotations());
    }

    public Command setStateCommand(State newState) {
        return Commands.runOnce(() -> setState(newState));
    }

    private void setIntakePosition(double position) {
        extenderMotor.setControl(new PositionDutyCycle(position));
    }

    private void setRollerSpeed(double speed) {
        rollerMotor.setControl(new DutyCycleOut(speed));
    }

}
