package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Crawler.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Crawler.Localization.Localizer;
import org.firstinspires.ftc.teamcode.Crawler.Navigation.Follower;
import org.firstinspires.ftc.teamcode.Crawler.Tasking.StateMachine;
import org.firstinspires.ftc.teamcode.Crawler.Tasking.Task;
import org.firstinspires.ftc.teamcode.Crawler.Tasking.TaskBuilder;

import java.util.List;


@Autonomous(name="Dynamic Curve Auto", group="Crawlier")
public class DynamicAuto extends LinearOpMode {
    Robot robot = new Robot();
    Localizer localizer = new Localizer(hardwareMap);
    Follower follower;
    StateMachine machine = new StateMachine();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        follower = new Follower(robot);
        TaskBuilder builder = new TaskBuilder(follower, localizer);

        // Build a dynamic path that arcs while spinning the shooters
        List<Task> scorePath = builder
                // 1. Arc toward the bar (Does not stop at waypoint)
                .waypoint(20, 15, 45, 4.0)

                // 2. Multitask: Rev shooters while moving to the next point
                .runAction(() -> robot.powerShooters(false))

                // 3. Arc through another point
                .waypoint(40, 5, -45, 4.0)

                // 4. Final precise drive (Stops here)
                .driveTo(60, 0, 0)

                // 5. Score action
                .runAction(() -> robot.indexer.setPower(1))
                .build();

        waitForStart();

        machine.start(scorePath);

        while (opModeIsActive()) {
            // Update GPS
            localizer.update();

            // Update State Machine
            machine.update();

            // Monitoring
            telemetry.addData("Status", machine.isBusy() ? "Running Path" : "Idle");
            telemetry.addData("X", localizer.x);
            telemetry.addData("Y", localizer.y);
            telemetry.update();
        }
    }
}