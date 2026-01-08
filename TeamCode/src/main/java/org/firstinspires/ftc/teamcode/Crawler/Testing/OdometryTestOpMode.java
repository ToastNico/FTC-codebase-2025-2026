package org.firstinspires.ftc.teamcode.Crawler.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Crawler.Localization.Localizer;

@Autonomous(name = "Odometry Test", group = "Crawler")
public class OdometryTestOpMode extends LinearOpMode {
    private Localizer localizer;
    private DcMotor leftMotor, rightMotor, strafeMotor;
    private ElapsedTime timer = new ElapsedTime();

    private static final double MOTOR_POWER = 0.3;
    private static final double EXPECTED_DISTANCE_MM = 1219.2;

    @Override
    public void runOpMode() throws InterruptedException {
        // Drive Motors
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        strafeMotor = hardwareMap.get(DcMotor.class, "strafeMotor");

        localizer = new Localizer(hardwareMap);

        telemetry.addLine("Ready for Odometry Test");
        telemetry.addLine("1. Place robot on line");
        telemetry.addLine("2. Press START");
        telemetry.update();

        waitForStart();

        testForwardDistance();
        stopMotors();
        sleep(2000);

        testRotation();
        stopMotors();
        sleep(2000);

        testStrafe();
        stopMotors();
    }

    private void testForwardDistance() {
        localizer.resetPose(0, 0, 0);
        timer.reset();

        leftMotor.setPower(MOTOR_POWER);
        rightMotor.setPower(MOTOR_POWER);

        while (opModeIsActive() && timer.seconds() < 10) {
            localizer.update();
            double x = localizer.getPoseX();
            double y = localizer.getPoseY();
            double currentDistance = Math.hypot(x, y);

            telemetry.addData("Test", "Forward 1219.2mm");
            telemetry.addData("Distance", "%.2f", currentDistance);
            telemetry.update();

            if (currentDistance >= EXPECTED_DISTANCE_MM) break;
        }
        stopMotors();
    }

    private void testRotation() {
        localizer.resetPose(0, 0, 0);
        timer.reset();

        // Turn in place
        leftMotor.setPower(-MOTOR_POWER);
        rightMotor.setPower(MOTOR_POWER);

        while (opModeIsActive() && timer.seconds() < 10) {
            localizer.update();
            double heading = localizer.getHeading();

            telemetry.addData("Test", "Rotate 360");
            telemetry.addData("Heading", "%.2f", heading);
            telemetry.update();

            if (Math.abs(heading) >= 355) break;
        }
        stopMotors();
    }

    private void testStrafe() {
        localizer.resetPose(0, 0, 0);
        timer.reset();

        strafeMotor.setPower(MOTOR_POWER);

        while (opModeIsActive() && timer.seconds() < 10) {
            localizer.update();
            // In your math, center wheel usually tracks Y (side to side)
            double yPos = localizer.getPoseY();

            telemetry.addData("Test", "Strafe 1219.2mm");
            telemetry.addData("Y Pos", "%.2f", yPos);
            telemetry.update();

            if (Math.abs(yPos) >= EXPECTED_DISTANCE_MM) break;
        }
        stopMotors();
    }

    private void stopMotors() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        strafeMotor.setPower(0);
    }
}