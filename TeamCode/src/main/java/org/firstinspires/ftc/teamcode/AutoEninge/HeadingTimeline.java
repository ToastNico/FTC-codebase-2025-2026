package org.firstinspires.ftc.teamcode.AutoEninge;

public class HeadingTimeline {
    // Stores our keyframes: <Percentage, TargetHeading>
    private final java.util.TreeMap<Double, Double> keyframes = new java.util.TreeMap<>();

    // User calls this to add points
    public HeadingTimeline at(double percentage, double heading) {
        // Clamp percentage between 0.0 and 1.0
        percentage = Math.max(0, Math.min(1.0, percentage));
        keyframes.put(percentage, heading);
        return this; // Allows chaining
    }

    // The Engine calls this to get the current target
    public double getTarget(double progress, double startHeading) {
        // Ensure we have a start point (0%) matching the robot's reality
        if (!keyframes.containsKey(0.0)) {
            keyframes.put(0.0, startHeading);
        }

        // Find the two keyframes we are currently between
        java.util.Map.Entry<Double, Double> floor = keyframes.floorEntry(progress);
        java.util.Map.Entry<Double, Double> ceiling = keyframes.ceilingEntry(progress);

        if (floor == null) return startHeading;
        if (ceiling == null) return floor.getValue(); // We passed the last point
        if (floor.equals(ceiling)) return floor.getValue(); // Exact match

        // Interpolate (Calculate the point in between)
        double range = ceiling.getKey() - floor.getKey();
        double progressInSegment = (progress - floor.getKey()) / range;

        double angleDiff = ceiling.getValue() - floor.getValue();

        // Handle wrapping (e.g. 10 -> 350 should be -20 degrees, not +340)
        while (angleDiff > 180) angleDiff -= 360;
        while (angleDiff < -180) angleDiff += 360;

        return floor.getValue() + (angleDiff * progressInSegment);
    }
}
