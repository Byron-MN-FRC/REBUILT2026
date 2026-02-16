package frc.robot;

import java.util.TreeMap;

public class DistanceVelocityMap {
    public static final TreeMap<Double, Integer> distanceVelocityMap = new TreeMap<>() {
        private static final long serialVersionUID = 1L;
        // 1. distance from target in meters
        // 2. velocity in rotations per minute (RPM)
        {
            put(0.0,2000);
            put(5.0, 2000);
            put(7.5, 2100);
            put(14.4, 2400);
            put(18.4, 2600);
        }
    };

    public static int getVelocity(double distance) {
        // Find the closest distance in the map that is less than or equal to the input distance
        Double closestDistance = distanceVelocityMap.floorKey(distance);
        if (closestDistance == null) {
            // If there is no such distance, return the velocity for the smallest distance
            return distanceVelocityMap.firstEntry().getValue();
        }
        return distanceVelocityMap.get(closestDistance);
    }
}
