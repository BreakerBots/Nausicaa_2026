package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.BreakerLib.util.factory.BreakerCANCoderFactory;
import frc.robot.BreakerLib.util.logging.BreakerLog;

public class Intake extends SubsystemBase {

    private boolean dynamicIntakeSpeed = false; // If true, roller speed scales with drivetrain velocity

    private final Drivetrain drivetrain;
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
    private double lastCommandedRollerSpeedDuty;
    private double lastCommandedRollerSpeedRps;

    public State state = State.STOWED;

    public Intake(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;


        // ---------- Pivot ----------

        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        //pivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        pivotConfig.CurrentLimits = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Constants.IntakeConstants.PIVOT_STATOR_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(Constants.IntakeConstants.PIVOT_SUPPLY_CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true);
                
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

        // PID
        pivotSlot0.kP = Constants.IntakeConstants.PIVOT_kP;
        pivotSlot0.kI = Constants.IntakeConstants.PIVOT_kI;
        pivotSlot0.kD = Constants.IntakeConstants.PIVOT_kD;

        pivotMotor.getConfigurator().apply(pivotConfig);

        targetPivotRotations = getPivotPositionRotations();
        pivotMotor.setControl(new MotionMagicDutyCycle(targetPivotRotations));

        // ---------- Roller ----------

        TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
        rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rollerConfig.CurrentLimits = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Constants.IntakeConstants.ROLLER_STATOR_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(Constants.IntakeConstants.ROLLER_SUPPLY_CURRENT_LIMIT)
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

        // Deploy extended with rollers idle before spinning up
        if (state == State.STOWED && newState == State.EXTENDED_INTAKING) {
            setState(State.EXTENDED_IDLE);
            setState(State.EXTENDED_INTAKING);
            return;
        }

        State previousState = state;
        state = newState;
        setIntakePosition(state.getRotation2d().getRotations());
        setRollerSpeed(computeRollerSpeedForState(state));

        BreakerLog.log("Intake/State/Previous", previousState.toString());
        BreakerLog.log("Intake/State/Current", state.toString());
        //BreakerLog.log("Intake/State/PivotPosition", state.getRotation2d().getRotations());
        //BreakerLog.log("Intake/State/RollerSpeed", state.getSpeed());
    }

    public Command setStateCommand(State newState) {
        return Commands.runOnce(() -> setState(newState), this);
    }
    
    // public void setPivotDutyCycle(double dutyCycle) {
    //     pivotMotor.setControl(new DutyCycleOut(dutyCycle));
    // }

    
    @Override
    public void periodic() {

        // Adjust roller speed dynamically to keep up with drivetrain
        if (dynamicIntakeSpeed && state == State.EXTENDED_INTAKING) {
            setRollerSpeed(computeRollerSpeedForState(state));
        }

        BreakerLog.log("Intake/RollerSpeedActual", rollerMotor.getVelocity().getValueAsDouble(), true); // What the motor is actual doing (after friction, load, limits, etc.)
        BreakerLog.log("Intake/RollerSpeedTargetDuty", lastCommandedRollerSpeedDuty, true); // Target duty cycle (-1 to 1)
        BreakerLog.log("Intake/RollerSpeedTargetRps", lastCommandedRollerSpeedRps, true); // Target velocity (rotations per second)
        BreakerLog.log("Intake/PivotTargetPosition", state.getRotation2d().getRotations(), true); // Setpoint, not from the encoder.
        BreakerLog.log("Intake/PivotEncoderAbsolutePosition", pivotEncoder.getAbsolutePosition().getValueAsDouble(), true); // CANcoder absolute position (magnet angle)
        BreakerLog.log("Intake/PivotEncoderPosition", this.getPivotPositionRotations(), true); // The one we actually use, compare to the Motion Magic target.
        if (BreakerLog.isVerboseLogging()) {
            BreakerLog.log("Electrical/Intake/roller", rollerMotor);
            BreakerLog.log("Electrical/Intake/pivot", pivotMotor);
        }
    }


    /** Pivot position from external encoder (rotations). */
    public double getPivotPositionRotations() {
        return pivotEncoder.getPosition().getValueAsDouble();
    }


    private void setIntakePosition(double positionRotations) {
        targetPivotRotations = positionRotations;
        if (BreakerLog.isVerboseLogging()) {
            BreakerLog.log("Intake/Status", "Setting intake position to " + targetPivotRotations + " rotations");
        }
        pivotMotor.setControl(new MotionMagicDutyCycle(targetPivotRotations));
    }


    /**
     * Computes roller speed for the given state. For intaking states (EXTENDED_INTAKING, STOW_INTAKING),
     * scales up with drivetrain forward velocity: roller does 2 rev in the time drivetrain travels
     * one roller circumference. Minimum is the state's base speed (never slower).
     * NOTE: Given our default roller speed of 0.7, we won't see this change unless we're moving close to 4mps
     */
    private double computeRollerSpeedForState(State state) {
        // Are we're using a static speed?
        if (!dynamicIntakeSpeed || state != State.EXTENDED_INTAKING) {
            return state.getSpeed();
        // Or are we calculating a dynamic speed based on the drivetrain?
        } else {
            double forwardSpeedMps = Math.max(0, drivetrain.getChassisSpeeds().vxMetersPerSecond); // No negative chasis speeds
            double rollerCircumference = Constants.IntakeConstants.ROLLER_CIRCUMFERENCE_METERS;
            double minDuty = Constants.IntakeConstants.SPEED_INTAKE; // Don't go under our minimum speed
            if (rollerCircumference <= 0) {
                return minDuty;
            }
            double velocityRps = 2.0 * forwardSpeedMps / rollerCircumference;
            double minRps = Math.abs(minDuty) * Constants.IntakeConstants.ROLLER_REV_PER_SEC_AT_FULL_DUTY;
            double targetRevPerSec = Math.max(minRps, velocityRps);
            double duty = -targetRevPerSec / Constants.IntakeConstants.ROLLER_REV_PER_SEC_AT_FULL_DUTY; // Convert roller rev/s back to a duty
            return MathUtil.clamp(duty, -1.0, minDuty); 
        }
    }

    /** Speed passed in as a duty cycle (-1 to 1). */
    private void setRollerSpeed(double speed) {
        lastCommandedRollerSpeedDuty = speed;
        if (speed != 0) {
            double velocityRps = speed * Constants.IntakeConstants.ROLLER_MOTOR_RPS_AT_FULL_DUTY;
            lastCommandedRollerSpeedRps = velocityRps;
            rollerMotor.setControl(new VelocityDutyCycle(velocityRps));
            //rollerMotor.setControl(new DutyCycleOut(-1.0));
            //rollerMotor.setControl(new DutyCycleOut(speed));
        } else {
            lastCommandedRollerSpeedRps = 0;
            rollerMotor.setControl(new DutyCycleOut(0));
        }
    }

}
