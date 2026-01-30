package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.BreakerLib.util.logging.BreakerLog;

public class Intake extends SubsystemBase {

    private final TalonFX pivotMotor = new TalonFX(Constants.IntakeConstants.PIVOT_MOTOR_ID,
            Constants.GeneralConstants.DRIVE_CANIVORE_BUS);
    private final TalonFX rollerMotor = new TalonFX(Constants.IntakeConstants.ROLLER_MOTOR_ID,
            Constants.GeneralConstants.DRIVE_CANIVORE_BUS);
    
    public State state = State.STOWED;

    /**
     * Intake states: pivot position (stowed/extended/jiggle) and roller speed (idle/intake/extake).
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
        setRollerSpeed(state.getSpeed());

        BreakerLog.log("Intake/State/Previous", previousState.toString());
        BreakerLog.log("Intake/State/Current", state.toString());
        BreakerLog.log("Intake/State/PivotPosition", state.getRotation2d().getRotations());
        BreakerLog.log("Intake/State/RollerSpeed", state.getSpeed());
    }

    public Command setStateCommand(State newState) {
        return Commands.runOnce(() -> setState(newState), this);
    }


    @Override
    public void periodic() {
        logStatus();
    }

    
    private void logStatus() {
        double pivotPosition = pivotMotor.getPosition().getValueAsDouble();
        double pivotVelocity = pivotMotor.getVelocity().getValueAsDouble();
        double rollerVelocity = rollerMotor.getVelocity().getValueAsDouble();
        String statusMessage = String.format("state=%s pivot=%.2frot %.1fvel roller=%.2fcmd %.1fvel",
                state, pivotPosition, pivotVelocity, state.getSpeed(), rollerVelocity);
        BreakerLog.log("Intake/Status", statusMessage);
    }

    private void setIntakePosition(double position) {
        pivotMotor.setControl(new PositionDutyCycle(position));
    }

    private void setRollerSpeed(double speed) {
        rollerMotor.setControl(new DutyCycleOut(speed));
    }

}
