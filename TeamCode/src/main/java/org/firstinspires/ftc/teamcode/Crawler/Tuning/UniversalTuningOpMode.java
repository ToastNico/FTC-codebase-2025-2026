package org.firstinspires.ftc.teamcode.Crawler.Tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Crawler.Localization.Localizer;

@TeleOp(name = "Crawler Universal Tuner", group = "Crawler")
public class UniversalTuningOpMode extends LinearOpMode {
    private Localizer localizer;
    private DcMotor leftMotor, rightMotor, strafeMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        leftMotor = hardwareMap.get(DcMotor.class, "frontLeft");
        rightMotor = hardwareMap.get(DcMotor.class, "backLeft");
        strafeMotor = hardwareMap.get(DcMotor.class, "frontRight");

        localizer = new Localizer(hardwareMap);
        localizer.resetPose(0, 0, 0);

        waitForStart();

        while (opModeIsActive()) {
            localizer.update();

            // --- MANUAL DRIVE (Default) ---
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            // Standard Mecanum/3-motor mix
            leftMotor.setPower(drive - rotate);
            rightMotor.setPower(drive + rotate);
            strafeMotor.setPower(strafe);

            // --- TELEMETRY ---
            telemetry.addLine("=== CRAWLER TUNING HUB ===");
            telemetry.addData("X (mm)", "%.2f", localizer.getPoseX());
            telemetry.addData("Y (mm)", "%.2f", localizer.getPoseY());
            telemetry.addData("Heading (Deg)", "%.2f", localizer.getHeading());

            telemetry.addLine("\n--- ENCODER RAW ---");
            telemetry.addData("L", localizer.getLeftEncoder());
            telemetry.addData("R", localizer.getRightEncoder());
            telemetry.addData("C", localizer.getStrafeEncoder());

            telemetry.addLine("\n--- QUICK CALIBRATION ---");
            telemetry.addLine("A: Reset Pose to 0");
            telemetry.addLine("X: Auto-Test Forward 1219.2mm");
            telemetry.addLine("Y: Auto-Test Rotate 360");

            if (gamepad1.a) localizer.resetPose(0, 0, 0);

            if (gamepad1.x) runForwardTest();
            if (gamepad1.y) runRotationTest();

            telemetry.update();
        }
    }

    private void runForwardTest() {
        localizer.resetPose(0, 0, 0);
        while (opModeIsActive() && localizer.getPoseX() < 1219.2) {
            localizer.update();
            leftMotor.setPower(0.3);
            rightMotor.setPower(0.3);
            telemetry.addData("Status", "Testing Forward...");
            telemetry.addData("Current X", localizer.getPoseX());
            telemetry.update();
        }
        stopMotors();
    }

    private void runRotationTest() {
        localizer.resetPose(0,0,0);
        while (opModeIsActive() && Math.abs(localizer.getHeading()) < 360) {
            localizer.update();
            leftMotor.setPower(-0.4);
            rightMotor.setPower(0.4);
            telemetry.addData("Status", "Testing Rotation...");
            telemetry.addData("Heading", localizer.getHeading());
            telemetry.update();
        }
        stopMotors();
    }

    private void stopMotors() {
        leftMotor.setPower(0); rightMotor.setPower(0); strafeMotor.setPower(0);
    }
}