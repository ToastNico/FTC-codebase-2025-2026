package org.firstinspires.ftc.teamcode.Crawler.Navigation;
import org.firstinspires.ftc.teamcode.Crawler.Core.PIDController;
import org.firstinspires.ftc.teamcode.Crawler.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Crawler.Localization.Localizer;

public class Follower {
    private Robot robot;
    public PIDController xPID = new PIDController(0.07, 0, 0.005, 0.02);
    public PIDController yPID = new PIDController(0.07, 0, 0.005, 0.02);
    public PIDController hPID = new PIDController(0.05, 0, 0.001, 0.01);

    public Follower(Robot robot) { this.robot = robot; }

    public boolean update(double tx, double ty, double th, double maxSpeed, Localizer loc) {
        double xErr = tx - loc.x;
        double yErr = ty - loc.y;
        double hErr = th - loc.getHeadingDegrees();

        // Field Centric Transformation
        double rads = -loc.getHeading();
        double rotX = xErr * Math.cos(rads) - yErr * Math.sin(rads);
        double rotY = xErr * Math.sin(rads) + yErr * Math.cos(rads);

        // Calculate raw PID outputs
        double drive = xPID.calculate(rotX, 0);
        double strafe = yPID.calculate(rotY, 0);
        double turn = hPID.calculate(hErr, 0);

        // Normalize and apply Velocity Limit (maxSpeed)
        double combined = Math.abs(drive) + Math.abs(strafe) + Math.abs(turn);
        double scale = Math.max(combined, 1.0 / maxSpeed);

        robot.powerDriveTrain(
                (drive + strafe + turn) / scale,
                (drive - strafe - turn) / scale,
                (drive - strafe + turn) / scale,
                (drive + strafe - turn) / scale
        );

        // Returns true only if we are settled (used for non-dynamic steps)
        return Math.abs(xErr) < 0.5 && Math.abs(yErr) < 0.5 && Math.abs(hErr) < 1.5;
    }
}