package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.AutoEninge.Sorter.robot;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.AutoEninge.MovementEngine;
import org.firstinspires.ftc.teamcode.AutoEninge.Team;

@TeleOp(name = "Camera MoveTo TeleOp")
public class CameraMoveToTeleop extends MovementEngine {

    double frontLeftPower;
    double frontRightPower;
    double backLeftPower;
    double backRightPower;


    boolean shootOn = false;
    boolean gobbleOn = false;

    boolean shootSequenceOn = false;

    @Override
    public void runPath() {
        boolean lastAPress = false;

        // TeleOp loop: allow driver to trigger moveToShoot with gamepad1.a
        while (opModeIsActive()) {
            // Keep the camera updated so detections are fresh

            // Simple one-press trigger for moveToShoot


            double leftY = gamepad1.left_stick_y;
            double rightY = gamepad1.right_stick_y;
            double leftX = gamepad1.left_stick_x;
            double rightX = gamepad1.right_stick_x;

            frontLeftPower = leftY + leftX;
            frontRightPower = rightY - rightX;
            backLeftPower = leftY - leftX;
            backRightPower = rightY + rightX;

            boolean aPressed = gamepad1.a;
            if (aPressed && !lastAPress) {
                // Call the existing moveToShoot routine (uses Team enum)
                moveToShoot(Team.BLUE); //TODO implement a dynamic way to change the team
            } else {
                robot.powerDriveTrain(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
            }
            lastAPress = aPressed;

            double forwards = gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

//            robot.drive(forwards, strafe, rotate);
            shootOn = gamepad2.right_bumper;
            robot.activateShooters(!shootOn);

            gobbleOn = gamepad2.left_bumper && !gobbleOn;
            robot.activateGobbler(gobbleOn);

            shootSequenceOn = gamepad2.triangle;
            if(shootSequenceOn){
                try {
                    frontLeftPower = 0;
                    frontRightPower = 0;
                    backLeftPower = 0;
                    backRightPower = 0;
                    robot.shootSequence();
                    shootSequenceOn = false;
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }

            // Telemetry for operator
            telemetry.addData("APress", aPressed);
            telemetry.addData("Detections", aprilTagWebcam.getDetectedTags() != null ? aprilTagWebcam.getDetectedTags().size() : 0);
            telemetry.update();
        }
    }
}
