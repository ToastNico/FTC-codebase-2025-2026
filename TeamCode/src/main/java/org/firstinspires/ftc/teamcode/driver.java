package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.AutoEninge.Robot;
import org.firstinspires.ftc.teamcode.Vision.AprilTagWebcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.opencv.core.Mat;

@TeleOp(name="Use ME! Driver")
public class driver extends OpMode{


        double frontLeftPower;
        double frontRightPower;
        double backLeftPower;
        double backRightPower;


        boolean shootOn = false;
        boolean gobbleOn = false;

        boolean shootSequenceOn = false;

    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();

        Robot robot;

        @Override
        public void init() {
            robot = new Robot(hardwareMap);

            aprilTagWebcam.init(hardwareMap, telemetry);
        }

        @Override
        public void loop() {
            aprilTagWebcam.update();

            AprilTagDetection id20 = aprilTagWebcam.getTagBySpecificId(20);
            aprilTagWebcam.displayDetectionTelemetry(id20);
            //telemetry.addData("id20 String", id20.toString());

            double leftY = gamepad1.left_stick_y;
            double rightY = gamepad1.right_stick_y;
            double leftX = gamepad1.left_stick_x;
            double rightX = gamepad1.right_stick_x;

            frontLeftPower = leftY + leftX;
            frontRightPower = rightY - rightX;
            backLeftPower = leftY - leftX;
            backRightPower = rightY + rightX;

            robot.powerDriveTrain(frontLeftPower, frontRightPower, backLeftPower, backRightPower);

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



        }


    }

