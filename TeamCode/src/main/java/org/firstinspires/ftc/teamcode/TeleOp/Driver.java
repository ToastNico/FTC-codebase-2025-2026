package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.AutoEninge.MovementEngine;
import org.firstinspires.ftc.teamcode.AutoEninge.Team;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name="Use ME! Driver (MovementEngine)")
public class Driver extends MovementEngine {

    // Logic states
    private boolean gobbleOn = false;
    private boolean lastLeftBumper = false;
    private boolean lastAPress = false;

    @Override
    public void runPath() throws InterruptedException {
        // Since MovementEngine calls runPath() inside runOpMode(),
        // the camera and robot are already initialized here.

        while (opModeIsActive()) {
            // 1. VISION UPDATE & TELEMETRY
            aprilTagWebcam.update();
            AprilTagDetection id20 = aprilTagWebcam.getTagBySpecificId(20);
            if (id20 != null) {
                aprilTagWebcam.displayDetectionTelemetry(id20);
            }

            // 2. DRIVE TRAIN CONTROL (Gamepad 1)
            double leftY = gamepad1.left_stick_y;
            double rightY = gamepad1.right_stick_y;
            double leftX = gamepad1.left_stick_x;
            double rightX = gamepad1.right_stick_x;

            // Manual driving
            double fl = leftY + leftX;
            double fr = rightY - rightX;
            double bl = leftY - leftX;
            double br = rightY + rightX;
            robot.powerDriveTrain(fl, fr, bl, br);

            // 3. AUTO-ALIGN TRIGGER (Gamepad 1 A)
            if (gamepad1.a && !lastAPress) {
                stop();
                moveToShoot(Team.BLUE);
            }
            lastAPress = gamepad1.a;

            // 4. MECHANISMS (Gamepad 2)

            // Shooter (Hold for Active)
            // Note: Keeping your original "!gamepad2.right_bumper" logic
            boolean shooterButtonHeld = gamepad2.right_bumper;
            robot.activateShooters(!shooterButtonHeld);

            // Gobbler (Toggle on Press)
            if (gamepad2.left_bumper && !lastLeftBumper) {
                gobbleOn = !gobbleOn;
                robot.activateGobbler(gobbleOn);
            }
            lastLeftBumper = gamepad2.left_bumper;

            // Shoot Sequence (Triangle)
            if (gamepad2.triangle) {
                stop();
                robot.shootSequence();
            }

            // Cycle Indexer (Dpad Up)
            if (gamepad2.dpad_up) {
                robot.cycleIndexer();
            }

            // 5. STATUS TELEMETRY
            telemetry.addData("Alpha Sensor", robot.ballColorSensor.alpha());
            telemetry.addData("Gobbler State", gobbleOn);
            telemetry.addLine("Christian likes feet");
            telemetry.update();
        }
    }
}