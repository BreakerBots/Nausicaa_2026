package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.BreakerLib.util.factory.BreakerCANCoderFactory;
import frc.robot.BreakerLib.util.logging.BreakerLog;

public class Intake extends SubsystemBase {

    private final TalonFX pivotMotor = new TalonFX(Constants.IntakeConstants.PIVOT_MOTOR_ID,
            Constants.GeneralConstants.SUPERSTRUCTURE_CANIVORE_BUS);
    private final TalonFX rollerMotor = new TalonFX(Constants.IntakeConstants.ROLLER_MOTOR_ID,
            Constants.GeneralConstants.SUPERSTRUCTURE_CANIVORE_BUS);
    private final CANcoder pivotEncoder = BreakerCANCoderFactory.createCANCoder(
            Constants.IntakeConstants.PIVOT_ENCODER_ID,
            Constants.GeneralConstants.SUPERSTRUCTURE_CANIVORE_BUS,
            Constants.IntakeConstants.PIVOT_ENCODER_DISCONTINUITY,
            Rotations.of(Constants.IntakeConstants.PIVOT_ENCODER_OFFSET_ROTATIONS),
            SensorDirectionValue.CounterClockwise_Positive);

    private double targetPivotRotations;

    public State state = State.STOWED;

    public Intake() {
        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotConfig.CurrentLimits = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Constants.IntakeConstants.PIVOT_STATOR_CURRENT_LIMIT)
                .withSupplyCurrentLimit(Constants.IntakeConstants.PIVOT_SUPPLY_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true);
                
        pivotConfig.Feedback.withRemoteCANcoder(pivotEncoder);

        Slot0Configs pivotSlot0 = pivotConfig.Slot0;


        // Motion Magic
        pivotConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.IntakeConstants.PIVOT_MM_CRUISE_VELOCITY;
        pivotConfig.MotionMagic.MotionMagicAcceleration = Constants.IntakeConstants.PIVOT_MM_ACCELERATION;
        pivotConfig.MotionMagic.MotionMagicJerk = Constants.IntakeConstants.PIVOT_MM_JERK;
         
        // Feedforward
        pivotSlot0.kS = Constants.IntakeConstants.PIVOT_kS;
        pivotSlot0.kG = Constants.IntakeConstants.PIVOT_kG;
        pivotSlot0.kV = Constants.IntakeConstants.PIVOT_kV;
        pivotSlot0.kA = Constants.IntakeConstants.PIVOT_kA;

        // // PID
        pivotSlot0.kP = Constants.IntakeConstants.PIVOT_kP;
        pivotSlot0.kI = Constants.IntakeConstants.PIVOT_kI;
        pivotSlot0.kD = Constants.IntakeConstants.PIVOT_kD;

        pivotMotor.getConfigurator().apply(pivotConfig);

        TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
        rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rollerConfig.CurrentLimits = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Constants.IntakeConstants.ROLLER_STATOR_CURRENT_LIMIT)
                .withStatorCurrentLimit(Constants.IntakeConstants.ROLLER_SUPPLY_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true);

        Slot0Configs rollerSlot0 = rollerConfig.Slot0;

        // Feedforward
        rollerSlot0.kV = Constants.IntakeConstants.ROLLER_kV;
        rollerSlot0.kS = Constants.IntakeConstants.ROLLER_kS;

        // PID
        rollerSlot0.kP = Constants.IntakeConstants.ROLLER_kP;
        rollerSlot0.kI = Constants.IntakeConstants.ROLLER_kI;
        rollerSlot0.kD = Constants.IntakeConstants.ROLLER_kD;

        rollerMotor.getConfigurator().apply(rollerConfig);
        
        targetPivotRotations = getPivotPositionRotations();
        pivotMotor.setControl(new MotionMagicDutyCycle(targetPivotRotations));
    }

    /** Pivot position from external encoder (rotations). */
    public double getPivotPositionRotations() {
        return pivotEncoder.getPosition().getValueAsDouble();
    }

    /**
     * Intake states: pivot position (stowed/extended/jiggle) and roller speed (idle/intake/extake).
     */
    public enum State {
        
        STOWED(Constants.IntakeConstants.POSITION_STOWED, Constants.IntakeConstants.SPEED_IDLE),
        EXTENDED_IDLE(Constants.IntakeConstants.POSITION_EXTENDED, Constants.IntakeConstants.SPEED_IDLE),
        EXTENDED_INTAKING(Constants.IntakeConstants.POSITION_EXTENDED, Constants.IntakeConstants.SPEED_INTAKE),
        EXTENDED_EXTAKING(Constants.IntakeConstants.POSITION_EXTENDED, Constants.IntakeConstants.SPEED_EXTAKE),
        FEED_JIGGLE_HIGH(Constants.IntakeConstants.POSITION_JIGGLE_HIGH, Constants.IntakeConstants.SPEED_FEED_JIGGLE),
        FEED_JIGGLE_LOW(Constants.IntakeConstants.POSITION_JIGGLE_LOW, Constants.IntakeConstants.SPEED_FEED_JIGGLE),
        STOW_INTAKING(Constants.IntakeConstants.POSITION_STOWED, Constants.IntakeConstants.SPEED_INTAKE);

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
        double pivotPosition = getPivotPositionRotations();
        double pivotVelocity = pivotMotor.getVelocity().getValueAsDouble();
        double rollerVelocity = rollerMotor.getVelocity().getValueAsDouble();
        String statusMessage = String.format("state=%s pivot=%.3frot tgt=%.3f %.1fvel roller=%.2fcmd %.1fvel",
                state, pivotPosition, targetPivotRotations, pivotVelocity, state.getSpeed(), rollerVelocity);
        BreakerLog.log("Intake/Status", statusMessage, true);
        BreakerLog.log("Intake/EncoderPosition", pivotEncoder.getAbsolutePosition().getValueAsDouble(), true);
        if (BreakerLog.isVerboseLogging()) {
            BreakerLog.log("Electrical/Intake/roller", rollerMotor);
            BreakerLog.log("Electrical/Intake/pivot", pivotMotor);
        }
    }

    private void setIntakePosition(double positionRotations) {
        targetPivotRotations = positionRotations;
        if (BreakerLog.isVerboseLogging()) {
            BreakerLog.log("Intake/Status", "Setting intake position to " + targetPivotRotations + " rotations");
        }
        pivotMotor.setControl(new MotionMagicDutyCycle(targetPivotRotations));
    }

    private void setRollerSpeed(double speed) {
        if (speed != 0) {
            rollerMotor.setControl(new DutyCycleOut(speed));
        }
        else {
            rollerMotor.setControl(new DutyCycleOut(0));
        }
    }

}
