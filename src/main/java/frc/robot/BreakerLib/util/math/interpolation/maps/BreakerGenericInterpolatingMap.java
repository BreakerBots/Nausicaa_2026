// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.math.interpolation.maps;

/**
 * Abstract base for maps that interpolate Y values from X keys.
 *
 * Keys must be Numbers (e.g., Double, Integer). Values are interpolated when
 * you query for a key that falls between known data points. Subclasses choose
 * the interpolation strategy (linear vs Lagrange polynomial).
 */
public abstract class BreakerGenericInterpolatingMap<K extends Number, V> extends java.util.AbstractMap<K, V> {

    /**
     * Returns the interpolated Y value for the given X key.
     *
     * If the key exactly matches a stored point, returns that point's value.
     * Otherwise, estimates the value using the map's interpolation strategy.
     *
     * @param interpolendValue the X value to look up (e.g., target distance in meters)
     * @return the estimated Y value (e.g., hood angle in degrees)
     */
    public abstract V getInterpolatedValue(K interpolendValue);

}
