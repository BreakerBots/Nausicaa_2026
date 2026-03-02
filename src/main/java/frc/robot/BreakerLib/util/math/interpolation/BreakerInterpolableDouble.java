// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.math.interpolation;

import edu.wpi.first.math.MathUtil;

/**
 * Wraps a single double for use with BreakerLib's interpolating maps.
 *
 * Use this when your Y values are simple scalars (e.g., hood angle in degrees,
 * flywheel RPM). The interpolating map will interpolate between neighboring
 * doubles when you query for a key that falls between known data points.
 */
public class BreakerInterpolableDouble implements BreakerInterpolable<BreakerInterpolableDouble> {
    private double value;

    public BreakerInterpolableDouble(double value) {
        this.value = value;
    }

    public double getValue() {
        return value;
    }

    @Override
    public BreakerInterpolableDouble interpolate(BreakerInterpolableDouble endValue, double t) {
        return new BreakerInterpolableDouble(MathUtil.interpolate(value, endValue.getValue(), t));
    }

    @Override
    public double[] getInterpolatableData() {
        return new double[] { value };
    }

    @Override
    public BreakerInterpolableDouble fromInterpolatableData(double[] interpolatableData) {
        return new BreakerInterpolableDouble(interpolatableData[0]);
    }

}
