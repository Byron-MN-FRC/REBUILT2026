package frc.robot;

import java.util.TreeMap;

public class DistanceVelocityMap {
    public static final TreeMap<Double, Integer> distanceVelocityMap = new TreeMap<>() {
        private static final long serialVersionUID = 1L;
        // 1. distance from target in feet
        // 2. velocity in rotations per minute (RPM)
        {
            // put(0.0, 2000);
            put(7.833, 1900); // 7' 10"
            // put(10.0, 2125); // close / climb position
            put(10.67, 2075);
            put(11.583, 2125); // 11' 7"
            // put(13.67, 2400); // 13' 8" (middle, i think)
            put(14.583, 2325);
            put(16.917, 2450);
            put(17.667, 2550);
            put(17.83, 2600); // 17' 10" (far)

        }
    };

    public static int getVelocity(double distance) {
        // // Find the closest distance in the map that is less than or equal to the
        // input distance
        return (int) calculateInterpolation(distance);
    }

    public static double calculateInterpolation(double distance) {

        if (distance < distanceVelocityMap.firstKey()) {
            return distanceVelocityMap.firstEntry().getValue();
        }

        if (distance > distanceVelocityMap.lastKey()) {
            return distanceVelocityMap.lastEntry().getValue();
        }


        var ceilDistance = distanceVelocityMap.ceilingKey(distance).doubleValue();
        var floorDistance = distanceVelocityMap.floorKey(distance).doubleValue();

        var ceilVelocity = distanceVelocityMap.ceilingEntry(distance).getValue().doubleValue();
        var floorVelocity = distanceVelocityMap.floorEntry(distance).getValue().doubleValue();
        
        var m = (ceilVelocity - floorVelocity) / (ceilDistance - floorDistance);
        var y = m * (distance - floorDistance) + floorVelocity;

        return y;
    }
}