package org.firstinspires.ftc.teamcode.AutoEninge;

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Robot {

    // --- 1. NEW CONSTANTS FOR REV CORE HEX MOTOR ---
    static final double COUNTS_PER_MOTOR_REV = 288.0; // REV Core Hex Motor
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing
    static final double COUNTS_PER_DEGREE = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / 360;

    DcMotor frontRight , frontLeft, backRight, backLeft;
    DcMotor shooterLeft, shooterRight;
    DcMotor indexer, gobbler;

    public ColorSensor ballColorSensor;
    Servo lifter;
    public IMU imu;

    int counter = 0;

    int indexerHome = 0;
    private static int alphaThreshold = 0;

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

        // --- 2. UPDATED INDEXER SETUP ---
        indexer = hwMap.get(DcMotor.class, "indexer");
        indexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Important: Reset encoder to 0 when robot starts
        indexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        indexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        if(stop) {
            shooterRight.setPower(0);
            shooterLeft.setPower(0);
        } else {
            shooterLeft.setPower(0.62);
            shooterRight.setPower(-0.62);
        }
    }

    public void powerDriveTrain(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower) {
        frontLeft.setPower(-frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(-backLeftPower);
        backRight.setPower(backRightPower);
    }

    public void activateGobbler(boolean gooble) {
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

    // --- 3. UPDATED PRECISE INDEXER METHOD ---
    public void rotateIndexer(double degrees, IndexerRotation direction) {
        int newTargetPosition;
        int currentPosition = indexer.getCurrentPosition();
        int tickMovement = (int) (degrees * COUNTS_PER_DEGREE);

        if (direction == IndexerRotation.COUNTERCLOCKWISE){
            newTargetPosition = currentPosition + tickMovement;
        } else {
            newTargetPosition = currentPosition - tickMovement;
        }

        indexer.setTargetPosition(newTargetPosition);
        indexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        indexer.setPower(0.5); // Set speed

        // Wait until motor is done moving
        while (indexer.isBusy()) {
            // efficient wait

        }

        indexer.setPower(0);
        indexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // --- 4. UPDATED SHOOT SEQUENCE (120 Degrees) ---
    public void shootSequence() throws InterruptedException {
        // 3 slots = 360 / 3 = 120 degrees
        double indexAngle = 120;

        rotateIndexer(60,IndexerRotation.CLOCKWISE);  // move from gobble indexer alignment to shooting alignment

        activateShooters(false);
        sleep(1500);

        // Shot 1
        sleep(1500);
        lifter.setPosition(-0.8);
        sleep(1000);
        lifter.setPosition(1);
        sleep(500);

        // Move to Slot 2
        rotateIndexer(indexAngle, IndexerRotation.CLOCKWISE);

        // Shot 2
        sleep(1500);
        lifter.setPosition(-0.8);
        sleep(1000);
        lifter.setPosition(1);
        sleep(500);

        // Move to Slot 3
        rotateIndexer(indexAngle, IndexerRotation.CLOCKWISE);

        // Shot 3
        sleep(1500);
        lifter.setPosition(-0.8);
        sleep(1000);
        lifter.setPosition(1);
        sleep(500);

        activateShooters(true);

        realignIndexer();
    }

    public void cycleIndexer() throws InterruptedException {
        // Cycles one slot (120 degrees)
        rotateIndexer(120, IndexerRotation.CLOCKWISE);
    }

    public void realignIndexer() {
        // Force the motor to go back to the saved "0" spot
        indexer.setTargetPosition(indexerHome);
        indexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        indexer.setPower(0.8); // Use higher power to ensure it snaps back firmly

        // Wait for it to get there
        while (indexer.isBusy()) {
            // waiting...
        }

        // OPTIONAL: Keep the motor powered to Hold position
        // If you set power to 0, it might slip again.
        // Leaving it in RUN_TO_POSITION with power holds it stiff.
        indexer.setPower(0);
        indexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public boolean isBallThere() {
        return ballColorSensor.alpha() <= alphaThreshold;
    }
}