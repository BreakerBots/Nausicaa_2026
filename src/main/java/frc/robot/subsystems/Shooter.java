package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.BreakerLib.util.logging.BreakerLog;

public class Shooter {
    private final TalonFX shooterFlywheel1Motor = new TalonFX(Constants.ShooterConstants.SHOOTER_FLYWHEEL_1_MOTOR_ID,
            Constants.GeneralConstants.SUPERSTRUCTURE_CANIVORE_BUS);

    private final TalonFX shooterFlywheel2Motor = new TalonFX(Constants.ShooterConstants.SHOOTER_FLYWHEEL_2_MOTOR_ID,
            Constants.GeneralConstants.SUPERSTRUCTURE_CANIVORE_BUS);

    private final TalonFX kickerMotor = new TalonFX(Constants.ShooterConstants.KICKER_MOTOR_ID,
            Constants.GeneralConstants.SUPERSTRUCTURE_CANIVORE_BUS);

    private final TalonFX trajectoryAdjusterMotor = new TalonFX(Constants.ShooterConstants.TRAJECTORY_ADJUSTER_MOTOR_ID,
            Constants.GeneralConstants.SUPERSTRUCTURE_CANIVORE_BUS);
    
    public State state = State.INACTIVE;

    /**
     * Right now, these states equate to arbitrary positions.
     * Ideally, they'd be based on actual modes for the arm (ie. intake, score,
     * stow, etc.)
     */
    public enum State {
        
        INACTIVE(Constants.ShooterConstants.SPEED_IDLE, Constants.ShooterConstants.SPEED_IDLE, Constants.ShooterConstants.SPEED_IDLE),
        SHOOTING(Constants.ShooterConstants.SPEED_FLYWHEEL_1_ACTIVE, Constants.ShooterConstants.SPEED_FLYWHEEL_2_ACTIVE, Constants.ShooterConstants.SPEED_KICKER_ACTIVE);

        private double flywheel1Speed;
        private double flywheel2Speed;
        private double kickerSpeed;


        private State(double flywheel1Speed, double flywheel2Speed, double kickerSpeed) {
             this.flywheel1Speed = flywheel1Speed;
             this.flywheel2Speed = flywheel2Speed;
             this.kickerSpeed = kickerSpeed;
        }

        public double getFlywheel1Speed() {
          return flywheel1Speed;
        }

        public double getFlywheel2Speed() {
          return flywheel2Speed;
        }

        public double getKickerSpeed() {
          return kickerSpeed;
        }

    }

    
    public void setState(State newState) {
        State previousState = state;
        state = newState;
        setFlywheel1Speed(state.getFlywheel1Speed());
        setFlywheel2Speed(state.getFlywheel2Speed());
        setKickerSpeed(state.getKickerSpeed());
        
        // Log state change
        // System.out.println("Arm state changed from " + previousState.toString() + " to " + state.toString());
        // BreakerLog.log("Arm/State/Previous", previousState.toString());
        // BreakerLog.log("Arm/State/Current", state.toString());
        // BreakerLog.log("Arm/State/Position", state.getRotation2d().getRotations());
    }

    public Command setStateCommand(State newState) {
        return Commands.runOnce(() -> setState(newState));
    }

    private void setFlywheel1Speed(double speed) {
        shooterFlywheel1Motor.setControl(new DutyCycleOut(speed));
    }

    private void setFlywheel2Speed(double speed) {
        shooterFlywheel2Motor.setControl(new DutyCycleOut(speed));
    }

    private void setKickerSpeed(double speed) {
        kickerMotor.setControl(new DutyCycleOut(speed));
    }

}
