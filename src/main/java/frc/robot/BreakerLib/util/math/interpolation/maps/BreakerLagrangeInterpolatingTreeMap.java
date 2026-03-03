// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.math.interpolation.maps;

import java.util.Map.Entry;

import edu.wpi.first.math.geometry.Translation2d;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeMap;

import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.BreakerLib.util.math.interpolation.BreakerInterpolable;

/**
 * Interpolating map that uses Lagrange polynomial interpolation over all data points.
 *
 * Fits a polynomial through every (X, Y) point and evaluates it at the query X.
 * Can capture non-linear curves better than linear interpolation, but may oscillate
 * with many points. Values must implement BreakerInterpolable (e.g., BreakerInterpolableDouble).
 */
public class BreakerLagrangeInterpolatingTreeMap<K extends Number, V extends BreakerInterpolable<V>>
         extends BreakerGenericInterpolatingMap<K, V> {
    private TreeMap<K, V> indexesAndValues;

    public BreakerLagrangeInterpolatingTreeMap(TreeMap<K, V> indexesAndValues) {
        this.indexesAndValues = indexesAndValues;
    }

    public BreakerLagrangeInterpolatingTreeMap() {
        indexesAndValues = new TreeMap<K, V>();
    }

    @Override
    public V getInterpolatedValue(K interpolendValue) {
        // Reference value used to call getInterpolatableData() and fromInterpolatableData()
        V refVal = indexesAndValues.pollFirstEntry().getValue();
        // For each component (e.g., hood angle, flywheel RPM), collect all (X, Y) points
        List<List<Double>> interpolatableVals = new ArrayList<>();
        double[] interpolatedValArr = new double[refVal.getInterpolatableData().length];
        for (int i = 0; i < refVal.getInterpolatableData().length; i++) {
            List<Double> intValPortion = new ArrayList<>();
            for (Entry<K, V> ent : indexesAndValues.entrySet()) {
                intValPortion.add(ent.getValue().getInterpolatableData()[i]);
            }
            interpolatableVals.add(intValPortion);
        }
        int j = 0;
        for (List<Double> listD : interpolatableVals) {
            // Build (X, Y) points for this component: X = map key, Y = component value
            Translation2d[] arr = new Translation2d[listD.size()];
            Iterator<K> it = indexesAndValues.keySet().iterator();
            int k = 0;
            while (it.hasNext()) {
                arr[k] = new Translation2d(it.next().doubleValue(), listD.get(k));
                k++;
            }
            // Lagrange interpolate this component at the query X, then reconstruct the result
            interpolatedValArr[j] = BreakerMath.interpolateLagrange(interpolendValue.doubleValue(), arr);
            j++;
        }
        return refVal.fromInterpolatableData(interpolatedValArr);
    }

    @Override
    public Set<Entry<K, V>> entrySet() {
        return indexesAndValues.entrySet();
    }

    @Override
    public V put(K key, V value) {
        return indexesAndValues.put(key, value);
    }

    @Override
    public void putAll(Map<? extends K, ? extends V> m) {
        indexesAndValues.putAll(m);
    }

    @Override
    public boolean containsKey(Object key) {
        return indexesAndValues.containsKey(key);
    }

    @Override
    public boolean containsValue(Object value) {
        return indexesAndValues.containsValue(value);
    }

    @Override
    public boolean replace(K key, V oldValue, V newValue) {
        return indexesAndValues.replace(key, oldValue, newValue);
    }
}
