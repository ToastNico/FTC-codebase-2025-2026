package org.firstinspires.ftc.teamcode.Teleop;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Crawler.Hardware.Robot;

@TeleOp(name="Use ME! Driver")
public class Driver extends OpMode {
    double frontLeftPower;
    double frontRightPower;
    double backLeftPower;
    double backRightPower;
    Robot robot = new Robot();

    boolean shootOn = false;
    boolean gobbleOn = false;

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        double leftY = gamepad1.left_stick_y;
        double rightY = gamepad1.right_stick_y;
        double leftX = gamepad1.left_stick_x;
        double rightX = gamepad1.right_stick_x;

        frontLeftPower = leftY + leftX;
        frontRightPower = rightY - rightX;
        backLeftPower = leftY - leftX;
        backRightPower = rightY + rightX;

        robot.powerDriveTrain(frontLeftPower, frontRightPower, backLeftPower, backRightPower);

        shootOn = gamepad2.right_bumper && !shootOn;
        gobbleOn = gamepad2.right_bumper && !gobbleOn;

        robot.powerShooters(!shootOn);
        robot.powerGobbler(!gobbleOn);
    }
}
