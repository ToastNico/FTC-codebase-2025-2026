package org.firstinspires.ftc.teamcode.Crawler.Localization;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Localizer {
    public double x = 0;
    public double y = 0;
    private double heading = 0; // Heading in Radians
    private int lastL, lastR, lastC;

    // Encoders (Dead Wheels)
    private final DcMotor leftEncoder, rightEncoder, centerEncoder;

    public static double TICKS_PER_MM = 74.51;
    public static double TRACK_WIDTH = 304.8;
    public static double CENTER_OFFSET = -101.6;

    public Localizer(HardwareMap hwMap) {
        // Ensure these names match your Hardware Configuration on the Driver Station
        leftEncoder = hwMap.get(DcMotor.class, "frontLeft");
        rightEncoder = hwMap.get(DcMotor.class, "backLeft");
        centerEncoder = hwMap.get(DcMotor.class, "frontRight");

        resetPose(0, 0, 0);
    }

    public void update() {
        int left = leftEncoder.getCurrentPosition();
        int right = rightEncoder.getCurrentPosition();
        int center = centerEncoder.getCurrentPosition();

        double dL = (left - lastL) / TICKS_PER_MM;
        double dR = (right - lastR) / TICKS_PER_MM;
        double dC = (center - lastC) / TICKS_PER_MM;

        // dt is change in heading (radians)
        double dt = (dR - dL) / TRACK_WIDTH;
        double dx = (dL + dR) / 2.0;
        double dy = dC - (CENTER_OFFSET * dt);

        // Update global coordinates based on heading
        x += dx * Math.cos(heading) - dy * Math.sin(heading);
        y += dx * Math.sin(heading) + dy * Math.cos(heading);
        heading += dt;

        lastL = left; lastR = right; lastC = center;
    }

    public void resetPose(double newX, double newY, double newHeadingDeg) {
        this.x = newX;
        this.y = newY;
        this.heading = Math.toRadians(newHeadingDeg);

        this.lastL = leftEncoder.getCurrentPosition();
        this.lastR = rightEncoder.getCurrentPosition();
        this.lastC = centerEncoder.getCurrentPosition();
    }

    public double getPoseX() { return x; }
    public double getPoseY() { return y; }

    /**
     * Returns the heading in degrees for telemetry and tuning.
     */
    public double getHeading() { return Math.toDegrees(heading); }

    /**
     * Returns raw encoder position for the Left dead wheel.
     */
    public int getLeftEncoder() {
        return leftEncoder.getCurrentPosition();
    }

    /**
     * Returns raw encoder position for the Right dead wheel.
     */
    public int getRightEncoder() {
        return rightEncoder.getCurrentPosition();
    }

    /**
     * Returns raw encoder position for the Center (Strafe) dead wheel.
     */
    public int getStrafeEncoder() {
        return centerEncoder.getCurrentPosition();
    }

    /**
     * Helper for navigation classes that specifically request degrees.
     */
    public double getHeadingDegrees() {
        return Math.toDegrees(heading);
    }
}