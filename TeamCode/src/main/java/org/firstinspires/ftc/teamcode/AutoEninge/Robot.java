package org.firstinspires.ftc.teamcode.AutoEninge;

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


public class Robot {

    DcMotor frontRight , frontLeft, backRight, backLeft;

    DcMotor shooterLeft, shooterRight;

    DcMotor indexer, gobbler;

    ColorSensor ballColorSensor;

    Servo lifter;

    IMU imu;

    volatile boolean pleaseWait = false;

    int TICKS_PER_REV_COREHEX = 288;

    public Robot(HardwareMap hwMap) {
        frontLeft = hwMap.get(DcMotor.class , "frontLeft");
        frontRight = hwMap.get(DcMotor.class , "frontRight");
        backRight = hwMap.get(DcMotor.class, "backRight");
        backLeft = hwMap.get(DcMotor.class, "backLeft");

        shooterLeft = hwMap.get(DcMotor.class, "leftShoot");
        shooterRight = hwMap.get(DcMotor.class, "rightShoot");

        shooterLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterRight.setDirection(DcMotorSimple.Direction.FORWARD);

        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        indexer = hwMap.get(DcMotor.class, "indexer");
        gobbler = hwMap.get(DcMotor.class, "gobbler");
        gobbler.setDirection(DcMotorSimple.Direction.REVERSE);

        ballColorSensor = hwMap.get(ColorSensor.class , "colorSensor");

        lifter = hwMap.get(Servo.class, "lifter");

        imu = hwMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));
    }

    public void activateShooters(boolean stop) {

        if(stop) {shooterRight.setPower(0); shooterLeft.setPower(0);} else {
            shooterLeft.setPower(0.55);
            shooterRight.setPower(-0.55);
        }

    }

    public void powerDriveTrain(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower) {
        frontLeft.setPower(-frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(-backLeftPower);
        backRight.setPower(backRightPower);
    }
    public void activateGobbler(boolean gooble) { //Feature not a bug!
        if (gooble) {
            gobbler.setPower(-1);
        } else {
            gobbler.setPower(0);
        }
    }

    public void drive(double forward, double strafe, double rotate) {
        double frontLeftPower = forward + strafe + rotate;
        double backLeftPower = forward - strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
        double backRightPower = forward + strafe - rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;

        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));

        powerDriveTrain(
                maxSpeed * (frontLeftPower / maxPower),
                maxSpeed * (frontRightPower / maxPower),
                maxSpeed * (backLeftPower / maxPower),
                maxSpeed * (backRightPower / maxPower)
        );

    }

    public void driveFieldRelative(double forward, double strafe, double rotate) {
        double theta = Math.atan2(forward, strafe);
        double r = Math.hypot(strafe, forward);

        theta = AngleUnit.normalizeRadians(
                theta - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)
        );

        double newForward = r * Math.sin(theta);
        double newStrafe = r * Math.cos(theta);

        this.drive(newForward, newStrafe, rotate);
    }

    public void rotateByAngle(DcMotorEx motor, double angle, double power) throws InterruptedException {
        int targetTicks = (int) ((angle / 360.0) * TICKS_PER_REV_COREHEX);
        int newTarget = motor.getCurrentPosition() + targetTicks;

        motor.setTargetPosition(newTarget);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(Math.abs(power));

       sleep(800);
        // Now that the loop is finished, it's safe to stop
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void shootSequence() throws InterruptedException {


        activateShooters(false);
        sleep(1000);
        for(int i = 0; i < 3; i++) {
            lifter.setPosition(-0.8);
            sleep(1000);
            lifter.setPosition(1);
            sleep(500);
            rotateByAngle((DcMotorEx) indexer, -120, 0.6);
            sleep(1000);


        }




        activateShooters(true);
    }

}
