package org.firstinspires.ftc.teamcode.Crawler.Core;


//import com.acmerobotics.dashboard.config.Config;
//
//@Config
public class PIDController {
    private final double kp, ki, kd, kf;
    private double lastError = 0, integral = 0;

    public PIDController(double kp, double ki, double kd, double kf) {
        this.kp = kp; this.ki = ki; this.kd = kd; this.kf = kf;
    }

    public double calculate(double target, double current) {
        double error = target - current;
        // Angle Wrapping: ensures the robot takes the shortest turn
        while (error > 180) error -= 360;
        while (error <= -180) error += 360;

        integral += error;
        double derivative = error - lastError;
        lastError = error;
        return (error * kp) + (integral * ki) + (derivative * kd) + (Math.signum(error) * kf);
    }
}