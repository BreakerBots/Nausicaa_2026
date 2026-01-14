// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.driverstation.gamepad.components;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.BreakerLib.driverstation.BreakerInputStream;
import java.util.function.BooleanSupplier;

/** Class which represents an analog HID trigger. */
public class BreakerAnalogTrigger implements BreakerInputStream, BooleanSupplier {
    private GenericHID hid;
    private int port;
    private double deadband = 0.0;
    private double threshold = 0.1; // Default threshold for button-like behavior
    private boolean invert;
    private Trigger trigger;

    /**
     * Construct uninverted trigger.
     * 
     * @param hid Controller.
     * @param analogTriggerAxisPort Axis port #.
     * @param invert Invert axis values.
     */
    public BreakerAnalogTrigger(GenericHID hid, int analogTriggerAxisPort) {
        this.hid = hid;
        port = analogTriggerAxisPort;
        invert = false;
        this.trigger = new Trigger(this);
    }

    /**
     * Construct trigger with invert.
     * 
     * @param hid Controller.
     * @param analogTriggerAxisPort Axis port #.
     * @param invert Invert axis values.
     */
    public BreakerAnalogTrigger(GenericHID hid, int analogTriggerAxisPort, boolean invert) {
        this.hid = hid;
        port = analogTriggerAxisPort;
        this.invert = invert;
        this.trigger = new Trigger(this);
    }

    /**
     * Deadband for trigger input. Equals 0 by default.
     * 
     * @param deadband Deadband value.
     */
    public void setDeadband(double deadband) {
        this.deadband = deadband;
    }

    /**
     * Set threshold for button-like behavior (when trigger is considered "pressed").
     * Default is 0.1 (10%).
     * 
     * @param threshold Threshold value (0.0 to 1.0).
     */
    public void setThreshold(double threshold) {
        this.threshold = MathUtil.clamp(threshold, 0.0, 1.0);
    }

    /**
     * Get the current threshold value.
     * 
     * @return Current threshold value.
     */
    public double getThreshold() {
        return threshold;
    }

    /** @return Trigger input without deadband. */
    public double getRaw() {
        return hid.getRawAxis(port);
    }

    /** @return Trigger input with deadband. */
    public double get() {
        return MathUtil.applyDeadband(getRaw(), deadband) * (invert ? -1 : 1);
    }

    @Override
    public double getAsDouble() {
        return get();
    }

    @Override
    public boolean getAsBoolean() {
        return get() > threshold;
    }

    /**
     * Get a Trigger object that can be used with whileTrue() and onFalse() methods.
     * 
     * @return Trigger object for button-like behavior.
     */
    public Trigger asTrigger() {
        return trigger;
    }

    /**
     * Convenience method for whileTrue() functionality.
     * 
     * @param command The command to run while the trigger is held.
     * @return The trigger object for chaining.
     */
    public Trigger whileTrue(edu.wpi.first.wpilibj2.command.Command command) {
        return trigger.whileTrue(command);
    }

    /**
     * Convenience method for onFalse() functionality.
     * 
     * @param command The command to run when the trigger is released.
     * @return The trigger object for chaining.
     */
    public Trigger onFalse(edu.wpi.first.wpilibj2.command.Command command) {
        return trigger.onFalse(command);
    }

    /**
     * Convenience method for onTrue() functionality.
     * 
     * @param command The command to run when the trigger is first pressed.
     * @return The trigger object for chaining.
     */
    public Trigger onTrue(edu.wpi.first.wpilibj2.command.Command command) {
        return trigger.onTrue(command);
    }
}
