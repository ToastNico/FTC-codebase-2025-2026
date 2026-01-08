package org.firstinspires.ftc.teamcode.Crawler.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {
    public DcMotor frontLeft, frontRight, backLeft, backRight;
    public DcMotor leftShooter, rightShooter, gobbler, indexer;

    public void init(HardwareMap hwMap) {
        frontLeft = hwMap.get(DcMotor.class, "frontLeft");
        frontRight = hwMap.get(DcMotor.class, "frontRight");
        backLeft = hwMap.get(DcMotor.class, "backLeft");
        backRight = hwMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        leftShooter = hwMap.get(DcMotor.class, "leftShoot");
        rightShooter = hwMap.get(DcMotor.class, "rightShoot");
        gobbler = hwMap.get(DcMotor.class, "gobbler");
        indexer = hwMap.get(DcMotor.class, "indexer");
    }

    public void powerDriveTrain(double fl, double fr, double bl, double br) {
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    public void powerShooters(boolean stop) {
        double power = stop ? 0 : 1;
        leftShooter.setPower(power);
        rightShooter.setPower(power);
    }

    public void powerGobbler(boolean stop) {
        double power = stop ? 0 : 1;
        gobbler.setPower(power);
    }
}