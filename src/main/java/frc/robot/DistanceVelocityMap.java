package frc.robot;

import java.util.TreeMap;

public class DistanceVelocityMap {
    public static final TreeMap<Double, Integer> distanceVelocityMap = new TreeMap<>() {
        private static final long serialVersionUID = 1L;
        // 1. distance from target in feet
        // 2. velocity in rotations per minute (RPM)
        {
            put(0.0, 2000);
            put(7.833, 2000); // 7' 10"
            put(10.0, 2125); // close / climb position
            put(11.583, 2125); // 11' 7"
            put(13.67, 2400); // 13' 8" (middle, i think)
            put(17.83, 2600); // 17' 10" (far)

        }
    };

    public static int getVelocity(double distance) {
        // // Find the closest distance in the map that is less than or equal to the
        // input distance
        Double closestDistance = distanceVelocityMap.floorKey(distance);
        if (closestDistance == null) {
        // // If there is no such distance, return the velocity for the smallest
        // distance
        return distanceVelocityMap.firstEntry().getValue();
        }
        return distanceVelocityMap.get(closestDistance);
        // return 2100; // Placeholder value, replace with actual implementation
    }
}