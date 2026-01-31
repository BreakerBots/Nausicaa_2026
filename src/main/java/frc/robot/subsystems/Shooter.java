package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.BreakerLib.util.logging.BreakerLog;

public class Shooter extends SubsystemBase {

    private final TalonFX shooterFlywheel1Motor = new TalonFX(Constants.ShooterConstants.SHOOTER_FLYWHEEL_1_MOTOR_ID,
            Constants.GeneralConstants.DRIVE_CANIVORE_BUS);
    private final TalonFX shooterFlywheel2Motor = new TalonFX(Constants.ShooterConstants.SHOOTER_FLYWHEEL_2_MOTOR_ID,
            Constants.GeneralConstants.DRIVE_CANIVORE_BUS);
    private final TalonFX shooterFlywheel3Motor = new TalonFX(Constants.ShooterConstants.SHOOTER_FLYWHEEL_3_MOTOR_ID,
            Constants.GeneralConstants.DRIVE_CANIVORE_BUS);

    private final TalonFX hoodMotor = new TalonFX(Constants.ShooterConstants.HOOD_MOTOR_ID,
            Constants.GeneralConstants.DRIVE_CANIVORE_BUS);

    public Shooter() {
        // Flywheels 2 and 3 follow flywheel 1 (same direction)
        int leaderId = Constants.ShooterConstants.SHOOTER_FLYWHEEL_1_MOTOR_ID;
        shooterFlywheel2Motor.setControl(new Follower(leaderId, MotorAlignmentValue.Aligned));
        shooterFlywheel3Motor.setControl(new Follower(leaderId, MotorAlignmentValue.Aligned));
    }


    public State state = State.INACTIVE;

    /** Shooter states: flywheel speeds (inactive vs shooting). */
    public enum State {
        
        INACTIVE(Constants.ShooterConstants.SPEED_IDLE, Constants.ShooterConstants.SPEED_IDLE, Constants.ShooterConstants.SPEED_IDLE),
        SHOOTING(Constants.ShooterConstants.SPEED_FLYWHEEL_1_ACTIVE, Constants.ShooterConstants.SPEED_FLYWHEEL_2_ACTIVE, Constants.ShooterConstants.SPEED_FLYWHEEL_3_ACTIVE);

        private double flywheel1Speed;
        private double flywheel2Speed;
        private double flywheel3Speed;

        private State(double flywheel1Speed, double flywheel2Speed, double flywheel3Speed) {
             this.flywheel1Speed = flywheel1Speed;
             this.flywheel2Speed = flywheel2Speed;
             this.flywheel3Speed = flywheel3Speed;
        }

        public double getFlywheel1Speed() {
          return flywheel1Speed;
        }

        public double getFlywheel2Speed() {
          return flywheel2Speed;
        }

        public double getFlywheel3Speed() {
          return flywheel3Speed;
        }


    }

    public void setState(State newState) {
        State previousState = state;
        state = newState;
        setFlywheel1Speed(state.getFlywheel1Speed());
        // Flywheels 2 and 3 follow flywheel 1 via Follower control in constructor

        BreakerLog.log("Shooter/State/Previous", previousState.toString());
        BreakerLog.log("Shooter/State/Current", state.toString());
        BreakerLog.log("Shooter/State/Flywheel1", state.getFlywheel1Speed());
    }

    public Command setStateCommand(State newState) {
        return Commands.runOnce(() -> setState(newState), this);
    }


    @Override
    public void periodic() {
        logStatus();
    }


    /** One compact line: state, flywheels vel/current. */
    private void logStatus() {
        double v1 = shooterFlywheel1Motor.getVelocity().getValueAsDouble();
        double v2 = shooterFlywheel2Motor.getVelocity().getValueAsDouble();
        double v3 = shooterFlywheel3Motor.getVelocity().getValueAsDouble();
        double hoodPos = hoodMotor.getPosition().getValueAsDouble();
        String line = String.format("state=%s f1=%.1fvel%.1fA f2=%.1fvel%.1fA f3=%.1fvel%.1fA feed=%.1fvel%.1fA hood=%.2frot",
                state, v1, v2, v3, hoodPos);
        BreakerLog.log("Shooter/Status", line);
    }

    private void setFlywheel1Speed(double speed) {
        shooterFlywheel1Motor.setControl(new DutyCycleOut(speed));
    }

}
