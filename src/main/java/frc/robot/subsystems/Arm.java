package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.BreakerLib.sensors.BreakerDigitalSensor;
import frc.robot.BreakerLib.util.logging.BreakerLog;

public class Arm extends SubsystemBase {

    // Driven by a single motor
    private final TalonFX armMotor = new TalonFX(Constants.ArmConstants.ARM_MOTOR_ID,
            Constants.GeneralConstants.SUPERSTRUCTURE_CANIVORE_BUS);
    private Roller roller;

    private final BreakerDigitalSensor beamBreak = BreakerDigitalSensor.fromDIO(Constants.ArmConstants.BEAM_BREAK_DIO_PORT, false);
    
    private Debouncer debouncer = new Debouncer(0.48);

    public State state = State.UP;

    /**
     * Right now, these states equate to arbitrary positions.
     * Ideally, they'd be based on actual modes for the arm (ie. intake, score,
     * stow, etc.)
     */
    public enum State {
        DOWN(Constants.ArmConstants.POSITION_DOWN),
        UP(Constants.ArmConstants.POSITION_UP),
        EXTAKE(Constants.ArmConstants.POSITION_EXTAKE),
        STOW(Constants.ArmConstants.POSITION_STOW);

        private Rotation2d rotation;

        private State(Rotation2d rotation) {
            this.rotation = rotation;
        }

        public Rotation2d getRotation2d() {
            return rotation;
        }
    }

    public Arm() {
        // Configure our arm motor
        TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();
        talonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        talonFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        talonFXConfig.CurrentLimits.StatorCurrentLimit = Constants.ArmConstants.ARM_CURRENT_LIMIT;  

        Slot0Configs slot0Configs = talonFXConfig.Slot0;

        // Motion Magic
        talonFXConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.ArmConstants.MM_CRUISE_VELOCITY;
        talonFXConfig.MotionMagic.MotionMagicAcceleration = Constants.ArmConstants.MM_ACCELERATION;
        talonFXConfig.MotionMagic.MotionMagicJerk = Constants.ArmConstants.MM_JERK;
        
        // Feedforward
        slot0Configs.kS = Constants.ArmConstants.kS;
        slot0Configs.kG = Constants.ArmConstants.kG;
        slot0Configs.kV = Constants.ArmConstants.kV;
        slot0Configs.kA = Constants.ArmConstants.kA;

        // PID
        slot0Configs.kP = Constants.ArmConstants.kP;
        slot0Configs.kI = Constants.ArmConstants.kI;
        slot0Configs.kD = Constants.ArmConstants.kD;

        armMotor.getConfigurator().apply(talonFXConfig);

        // Zeroes: arm needs to be physically straight up or code will be off
        armMotor.setPosition(0.0);

        setupLogging();
    }

    private void setupLogging() {
        SmartDashboard.putData("Arm", new Sendable() {

            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("Arm Angle");
                    
                builder.addDoubleProperty("Angle", () -> armMotor.getPosition().getValueAsDouble(), null);
            }
            
        });
    }

    public Command homePosition() {
        return Commands.runOnce(() -> armMotor.setPosition(0.0));
    }

    public void setRoller(Roller roller) {
        this.roller = roller;
    }

    public void setState(State newState) {
        State previousState = state;
        state = newState;
        setArmPosition(state.getRotation2d().getRotations());
        
        // Log state change
        System.out.println("Arm state changed from " + previousState.toString() + " to " + state.toString());
        BreakerLog.log("Arm/State/Previous", previousState.toString());
        BreakerLog.log("Arm/State/Current", state.toString());
        BreakerLog.log("Arm/State/Position", state.getRotation2d().getRotations());
    }

    public Command setStateCommand(State newState) {
        return Commands.runOnce(() -> setState(newState));
    }

    private void setArmPosition(double position) {
        armMotor.setControl(new PositionDutyCycle(position));
    }

    public void setVoltageOutput(double output) {
        armMotor.setControl(new DutyCycleOut(output));
    }


    @Override
    public void periodic() {

       if (state == State.DOWN && debouncer.calculate(beamBreak.isTriggered())) {
            BreakerLog.log("Arm/BeamBreakEvent", "Triggered!");
            roller.setState(Roller.State.ALGAE_STOW);
            setState(State.STOW); 
        }

        if (roller.getState() == Roller.State.ALGAE_EXTAKE && state == State.UP && debouncer.calculate(!beamBreak.isTriggered())) {
            // BreakerLog.log("Arm/BeamBreakEvent", "Triggered!");
            roller.setState(Roller.State.IDLE);
            setState(State.UP); 
        }

        //if (debouncer.calculate(beamBreak.isTriggered())) {
        //    BreakerLog.log("Arm/BeamBreakEvent", "Triggered!");
        //    System.out.println("Triggered!");
        //}
    } 
}
