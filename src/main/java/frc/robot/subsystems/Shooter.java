package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.BreakerLib.util.logging.BreakerLog;

public class Shooter extends SubsystemBase {

    private final TalonFX shooterFlywheel1Motor = new TalonFX(Constants.ShooterConstants.SHOOTER_FLYWHEEL_1_MOTOR_ID,
            Constants.GeneralConstants.SUPERSTRUCTURE_CANIVORE_BUS);
    private final TalonFX shooterFlywheel2Motor = new TalonFX(Constants.ShooterConstants.SHOOTER_FLYWHEEL_2_MOTOR_ID,
            Constants.GeneralConstants.SUPERSTRUCTURE_CANIVORE_BUS);
    private final TalonFX shooterFlywheel3Motor = new TalonFX(Constants.ShooterConstants.SHOOTER_FLYWHEEL_3_MOTOR_ID,
            Constants.GeneralConstants.SUPERSTRUCTURE_CANIVORE_BUS);

    public Shooter(TrajectoryManager trajectoryManager) {
        this.trajectoryManager = trajectoryManager;
        TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
        flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        flywheelConfig.CurrentLimits = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Constants.ShooterConstants.FLYWHEEL_STATOR_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(Constants.ShooterConstants.FLYWHEEL_SUPPLY_CURRENT_LIMIT)
                .withSupplyCurrentLimitEnable(true);
        Slot0Configs slot0 = flywheelConfig.Slot0;
        
        slot0.kS = Constants.ShooterConstants.SHOOTER_kS;
        slot0.kV = Constants.ShooterConstants.SHOOTER_kV;
        //slot0.kA = Constants.ShooterConstants.SHOOTER_kA;
        slot0.kP = Constants.ShooterConstants.SHOOTER_kP;
        slot0.kI = Constants.ShooterConstants.SHOOTER_kI;
        slot0.kD = Constants.ShooterConstants.SHOOTER_kD;

        shooterFlywheel1Motor.getConfigurator().apply(flywheelConfig);
        shooterFlywheel2Motor.getConfigurator().apply(flywheelConfig);
        shooterFlywheel3Motor.getConfigurator().apply(flywheelConfig);
    }


    public State state = State.INACTIVE;

    private final TrajectoryManager trajectoryManager;

    /** Shooter states (flywheel speed is separate, via getFlywheelSpeed). */
    public enum State { INACTIVE, SPINNING_UP, SHOOTING }

    /** Returns the flywheel speed (rotations/sec) for the given state. Uses TrajectoryManager for distance lookup. */
    public double getFlywheelSpeed(State state) {
         if (state == State.INACTIVE) {
             return Constants.ShooterConstants.SPEED_IDLE;
         } else {
             return trajectoryManager.getFlywheelSpeedForDistance();
         }
        // return switch (state) {
        //     case INACTIVE -> Constants.ShooterConstants.SPEED_IDLE;
        //     case SPINNING_UP, SHOOTING -> Constants.ShooterConstants.SPEED_FLYWHEEL_ACTIVE;
        // };
    }

    public void setState(State newState) {
        State previousState = state;
        state = newState;
        setFlywheelSpeed(getFlywheelSpeed(state));

        BreakerLog.log("Shooter/State/Previous", previousState.toString());
        BreakerLog.log("Shooter/State/Current", state.toString());
    }

    public Command setStateCommand(State newState) {
        return Commands.runOnce(() -> setState(newState), this);
    }

    /** Returns true when all flywheels are within tolerance of the active target speed. */
    public boolean isAtTargetSpeed() {
        double target = getFlywheelSpeed(state);
        if (target == 0) return true;
        double tolerance = Math.abs(target) * Constants.ShooterConstants.FLYWHEEL_SPEED_TOLERANCE;
        double v1 = shooterFlywheel1Motor.getVelocity().getValueAsDouble();
        double v2 = shooterFlywheel2Motor.getVelocity().getValueAsDouble();
        double v3 = shooterFlywheel3Motor.getVelocity().getValueAsDouble();
        return Math.abs(v1 - target) <= tolerance
                && Math.abs(v2 - target) <= tolerance
                && Math.abs(v3 - target) <= tolerance;
    }

    @Override
    public void periodic() {

        // Update the flywheel speed continuously based on our distance to target
        if (state == State.SPINNING_UP || state == State.SHOOTING) {
            setFlywheelSpeed(getFlywheelSpeed(state));
        }

        double v1 = shooterFlywheel1Motor.getVelocity().getValueAsDouble();
        double v2 = shooterFlywheel2Motor.getVelocity().getValueAsDouble();
        double v3 = shooterFlywheel3Motor.getVelocity().getValueAsDouble();
        double distanceToTarget = trajectoryManager.getDistanceToTarget();
        double targetFlywheelSpeed = getFlywheelSpeed(state);
        BreakerLog.log("Shooter/DistanceToTarget", distanceToTarget, true);
        BreakerLog.log("Shooter/FlywheelTargetSpeed", targetFlywheelSpeed, true);
        BreakerLog.log("Shooter/Flywheel1Speed", v1, true);
        if (BreakerLog.isVerboseLogging()) {
            BreakerLog.log("Shooter/Flywheel2Speed", v2);
            BreakerLog.log("Shooter/Flywheel3Speed", v3);
            BreakerLog.log("Electrical/Shooter/flywheel1", shooterFlywheel1Motor);
            BreakerLog.log("Electrical/Shooter/flywheel2", shooterFlywheel2Motor);
            BreakerLog.log("Electrical/Shooter/flywheel3", shooterFlywheel3Motor);
        }
    }


    private void setFlywheelSpeed(double speed) {
        if (speed == 0) {
            DutyCycleOut coast = new DutyCycleOut(0).withOverrideBrakeDurNeutral(false);
            shooterFlywheel1Motor.setControl(coast);
            shooterFlywheel2Motor.setControl(coast);
            shooterFlywheel3Motor.setControl(coast);
        } else {
            VelocityVoltage velocityControl = new VelocityVoltage(speed).withAcceleration(400);
            shooterFlywheel1Motor.setControl(velocityControl);
            shooterFlywheel2Motor.setControl(velocityControl);
            shooterFlywheel3Motor.setControl(velocityControl);
        }
    }

}
