package org.firstinspires.ftc.teamcode.Crawler.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.List;

import org.firstinspires.ftc.teamcode.Crawler.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Crawler.Localization.Localizer;
import org.firstinspires.ftc.teamcode.Crawler.Navigation.Follower;
import org.firstinspires.ftc.teamcode.Crawler.Tasking.*;

@Autonomous(name = "Path Following Test", group = "Crawler")
public class PathFollowingTest extends LinearOpMode {
    Robot robot = new Robot();
    Localizer localizer;
    Follower follower;
    StateMachine machine = new StateMachine();

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize Hardware
        robot.init(hardwareMap);
        localizer = new Localizer(hardwareMap);
        follower = new Follower(robot);

        TaskBuilder builder = new TaskBuilder(follower, localizer);

        // Define a "Test Circuit"
        // 1. Move 609.6mm forward with a slight turn (Dynamic)
        // 2. Arc through the center (Dynamic)
        // 3. Finish at (1219.2, 0) facing forward (Precise)
        List<Task> testPath = builder
                .waypoint(609.6, 304.8, 45, 101.6, 0.7)  // Speed limited to 70%
                .waypoint(914.4, -304.8, -45, 101.6, 0.7)
                .driveTo(1219.2, 0, 0, 0.5)          // Final approach at 50% speed
                .build();

        telemetry.addLine("Ready to test path following");
        telemetry.addLine("Robot will perform an S-Curve");
        telemetry.update();

        waitForStart();

        // Start the sequence
        machine.start(testPath);

        while (opModeIsActive() && machine.isBusy()) {
            // Update GPS
            localizer.update();

            // Update Path Following Logic
            machine.update();

            // Monitor Progress
            telemetry.addData("Status", "Executing Path...");
            telemetry.addData("Current X", "%.2f", localizer.getPoseX());
            telemetry.addData("Current Y", "%.2f", localizer.getPoseY());
            telemetry.addData("Heading", "%.2f", localizer.getHeading());
            telemetry.update();
        }

        // Safety stop
        robot.powerDriveTrain(0,0,0,0);
        telemetry.addLine("Path Complete!");
        telemetry.update();
        sleep(2000);
    }
}