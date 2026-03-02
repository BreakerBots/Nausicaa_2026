// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.math.interpolation;

import edu.wpi.first.math.interpolation.Interpolatable;
import frc.robot.BreakerLib.util.math.BreakerMath;

/**
 * Extends WPILib's Interpolatable interface to support non-linear interpolation.
 *
 * WPILib's Interpolatable only provides linear interpolation between two values
 * given a blend factor t. BreakerInterpolable adds the ability to interpolate
 * using a query key and bounding keys, which enables map-based lookups where
 * the system computes t from the key position.
 *
 * Implementations must provide:
 * - getInterpolatableData: expose raw numeric components for polynomial-style interpolation
 * - fromInterpolatableData: reconstruct an instance from those components
 */
public interface BreakerInterpolable<V> extends Interpolatable<V> {

    public abstract double[] getInterpolatableData();

    public abstract V fromInterpolatableData(double[] interpolatableData);

    public default V interpolate(double query, double lowKey, double highKey, V highVal) {
        return interpolate(highVal, BreakerMath.getLerpT(query, lowKey, highKey));
    }
}
