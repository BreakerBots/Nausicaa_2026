package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.BreakerLib.util.logging.BreakerLog;

public class Hopper extends SubsystemBase {
    
    private final TalonFX indexerMotor = new TalonFX(Constants.HopperConstants.HOPPER_MOTOR_ID,
            Constants.GeneralConstants.SUPERSTRUCTURE_CANIVORE_BUS);
    private final TalonFX feederMotor = new TalonFX(Constants.HopperConstants.FEEDER_MOTOR_ID,
            Constants.GeneralConstants.SUPERSTRUCTURE_CANIVORE_BUS);

    public Hopper() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Constants.HopperConstants.STATOR_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true);
        indexerMotor.getConfigurator().apply(config);
        feederMotor.getConfigurator().apply(config);
    }

    public State state = State.INACTIVE;

    /** Hopper states: indexer speed (inactive vs indexing), feeder states (inactive vs feeding). */
    public enum State {
 
        INACTIVE(Constants.HopperConstants.SPEED_INACTIVE, Constants.HopperConstants.SPEED_INACTIVE),
        FEEDING(Constants.HopperConstants.SPEED_INDEXING, Constants.HopperConstants.SPEED_FEEDING),
        AGITATING(Constants.HopperConstants.SPEED_INDEXING, Constants.HopperConstants.SPEED_INACTIVE);

        private double indexerSpeed;
        private double feederSpeed;

        private State(double indexerSpeed, double feederSpeed) {
             this.indexerSpeed = indexerSpeed;
             this.feederSpeed = feederSpeed;
        }

        public double getIndexerSpeed() {
          return indexerSpeed;
        }

        public double getFeederSpeed() {
          return feederSpeed;
        }


    }
    
    public void setState(State newState) {
        State previousState = state;
        state = newState;
        setIndexerSpeed(state.getIndexerSpeed());
        setFeederSpeed(state.getFeederSpeed());

        BreakerLog.log("Hopper/State/Previous", previousState.toString());
        BreakerLog.log("Hopper/State/Current", state.toString());
        BreakerLog.log("Hopper/State/IndexerSpeed", state.getIndexerSpeed());
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
        double vel = indexerMotor.getVelocity().getValueAsDouble();
        double cur = indexerMotor.getStatorCurrent().getValueAsDouble();
        String line = String.format("state=%s roller=%.2fcmd %.1fvel %.1fA",
                state, state.getIndexerSpeed(), vel, cur);
        BreakerLog.log("Hopper/Status", line);
        BreakerLog.log("Electrical/Hopper/indexer", indexerMotor);
        BreakerLog.log("Electrical/Hopper/feeder", feederMotor);
    }

    private void setIndexerSpeed(double speed) {
        indexerMotor.setControl(new DutyCycleOut(speed));
    }

    private void setFeederSpeed(double speed) {
        feederMotor.setControl(new DutyCycleOut(speed));
    }

}
