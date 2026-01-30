package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

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

    private final TalonFX feederMotor = new TalonFX(Constants.ShooterConstants.FEEDER_MOTOR_ID,
            Constants.GeneralConstants.DRIVE_CANIVORE_BUS);

    private final TalonFX hoodMotor = new TalonFX(Constants.ShooterConstants.HOOD_MOTOR_ID,
            Constants.GeneralConstants.DRIVE_CANIVORE_BUS);
    
    public State state = State.INACTIVE;

    /** Shooter states: flywheel and feeder speeds (inactive vs shooting). */
    public enum State {
        
        INACTIVE(Constants.ShooterConstants.SPEED_IDLE, Constants.ShooterConstants.SPEED_IDLE, Constants.ShooterConstants.SPEED_IDLE, Constants.ShooterConstants.SPEED_IDLE),
        SHOOTING(Constants.ShooterConstants.SPEED_FLYWHEEL_1_ACTIVE, Constants.ShooterConstants.SPEED_FLYWHEEL_2_ACTIVE, Constants.ShooterConstants.SPEED_FLYWHEEL_3_ACTIVE, Constants.ShooterConstants.SPEED_KICKER_ACTIVE);

        private double flywheel1Speed;
        private double flywheel2Speed;
        private double flywheel3Speed;
        private double feederSpeed;

        private State(double flywheel1Speed, double flywheel2Speed, double flywheel3Speed, double feederSpeed) {
             this.flywheel1Speed = flywheel1Speed;
             this.flywheel2Speed = flywheel2Speed;
             this.flywheel3Speed = flywheel3Speed;
             this.feederSpeed = feederSpeed;
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

        public double getFeederSpeed() {
          return feederSpeed;
        }

    }

    public void setState(State newState) {
        State previousState = state;
        state = newState;
        setFlywheel1Speed(state.getFlywheel1Speed());
        setFlywheel2Speed(state.getFlywheel2Speed());
        setFlywheel3Speed(state.getFlywheel3Speed());
        setFeederSpeed(state.getFeederSpeed());

        BreakerLog.log("Shooter/State/Previous", previousState.toString());
        BreakerLog.log("Shooter/State/Current", state.toString());
        BreakerLog.log("Shooter/State/Flywheel1", state.getFlywheel1Speed());
        BreakerLog.log("Shooter/State/Feeder", state.getFeederSpeed());
    }

    public Command setStateCommand(State newState) {
        return Commands.runOnce(() -> setState(newState), this);
    }


    @Override
    public void periodic() {
        logStatus();
    }


    /** One compact line: state, flywheels + feeder vel/current. */
    private void logStatus() {
        double v1 = shooterFlywheel1Motor.getVelocity().getValueAsDouble();
        double v2 = shooterFlywheel2Motor.getVelocity().getValueAsDouble();
        double v3 = shooterFlywheel3Motor.getVelocity().getValueAsDouble();
        double vFeed = feederMotor.getVelocity().getValueAsDouble();
        double a1 = shooterFlywheel1Motor.getStatorCurrent().getValueAsDouble();
        double a2 = shooterFlywheel2Motor.getStatorCurrent().getValueAsDouble();
        double a3 = shooterFlywheel3Motor.getStatorCurrent().getValueAsDouble();
        double aFeed = feederMotor.getStatorCurrent().getValueAsDouble();
        double hoodPos = hoodMotor.getPosition().getValueAsDouble();
        String line = String.format("state=%s f1=%.1fvel%.1fA f2=%.1fvel%.1fA f3=%.1fvel%.1fA feed=%.1fvel%.1fA hood=%.2frot",
                state, v1, a1, v2, a2, v3, a3, vFeed, aFeed, hoodPos);
        BreakerLog.log("Shooter/Status", line);
    }

    private void setFlywheel1Speed(double speed) {
        shooterFlywheel1Motor.setControl(new DutyCycleOut(speed));
    }

    private void setFlywheel2Speed(double speed) {
        shooterFlywheel2Motor.setControl(new DutyCycleOut(speed));
    }

    private void setFlywheel3Speed(double speed) {
        shooterFlywheel3Motor.setControl(new DutyCycleOut(speed));
    }

    private void setFeederSpeed(double speed) {
        feederMotor.setControl(new DutyCycleOut(speed));
    }
}
