package org.firstinspires.ftc.teamcode.AutoEninge;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot {

    DcMotor frontRight , frontLeft, backRight, backLeft;

    DcMotor shooterLeft, shooterRight;

    DcMotor indexer, gobbler;

    ColorSensor ballColorSensor;

    Servo lifter;

    public Robot(HardwareMap hwMap) {
        frontLeft = hwMap.get(DcMotor.class , "frontLeft");
        frontRight = hwMap.get(DcMotor.class , "frontRight");
        backRight = hwMap.get(DcMotor.class, "backRight");
        backLeft = hwMap.get(DcMotor.class, "backLeft");

        shooterLeft = hwMap.get(DcMotor.class, "leftShoot");
        shooterRight = hwMap.get(DcMotor.class, "rightShoot");

        shooterLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterRight.setDirection(DcMotorSimple.Direction.FORWARD);

        indexer = hwMap.get(DcMotor.class, "indexer");
        gobbler = hwMap.get(DcMotor.class, "gobbler");

        ballColorSensor = hwMap.get(ColorSensor.class , "ballCSensor");

        lifter = hwMap.get(Servo.class, "lifter");
    }

    public void activateShooters(boolean stop) {
        if(stop) {shooterRight.setPower(0); shooterLeft.setPower(0);} else {
            shooterLeft.setPower(0.03);
            shooterRight.setPower(0.03);
        }

    }

}
