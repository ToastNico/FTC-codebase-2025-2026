package org.firstinspires.ftc.teamcode.AutoEninge;

import static org.firstinspires.ftc.teamcode.AutoEninge.MovementEngine.AutoEngineConfig.Kp;
import static org.firstinspires.ftc.teamcode.AutoEninge.MovementEngine.AutoEngineConfig.MIN_POWER;
import static org.firstinspires.ftc.teamcode.AutoEninge.MovementEngine.AutoEngineConfig.TICKS_PER_METER;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Vision.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.Vision.Rotation;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public abstract class MovementEngine extends LinearOpMode {

    /**
    * Internal configuration class, primarily to allow for simpler configuration
    */
    @Config
    public static class AutoEngineConfig {
        public static final double ODO_WHEEL_DIAMETER_METERS = 0.048;
        public static final double ENCODER_TICKS_PER_REV = 2000;
        public static final double ODO_WHEEL_CIRCUMFERENCE = ODO_WHEEL_DIAMETER_METERS * Math.PI;
        public static final double TICKS_PER_METER = (ENCODER_TICKS_PER_REV / ODO_WHEEL_CIRCUMFERENCE);

        public static double Kp = 0.6;
        public static double Kd = 0;
        public static double Ki = 0.0015;

        // FIXED: Set strafe coefficients to non-zero values
        public static double strafe_Kp = 0.6;
        public static double strafe_Ki = 0.0015;
        public static double strafe_Kd = 0;

        public static final double STEER_P = 0.02;
        public static final double MIN_POWER = 0.1;
    }

    protected DcMotor backLeft, backRight, frontLeft, frontRight;
    protected DcMotor leftOdo, rightOdo, centerOdo;
    protected IMU imu;
    protected AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();

    private double alpha = 0;
    private double beta = 0;
    private double gamma = 0;
    private double theta = 0;
    private double aSide = 0;
    private double bSide = 0;
    private final double cSide = 150;

    public abstract void runPath();

    @Override
    public void runOpMode() throws InterruptedException {
        aprilTagWebcam.init(hardwareMap, telemetry);

        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");

        leftOdo = hardwareMap.get(DcMotor.class, "frontLeft");
        rightOdo = hardwareMap.get(DcMotor.class, "backRight");
        centerOdo = hardwareMap.get(DcMotor.class, "backLeft");

        setMotorBehavior();

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));

        resetOdometry();
        imu.resetYaw();
        waitForStart();
        imu.resetYaw();

        if (opModeIsActive()) {
            aprilTagWebcam.update();
            runPath();
        }
    }

    /**
     * THis function allows the robot to move to the remade shooting position
     * @param team the team to move to*/
    public void moveToShoot(Team team) {
        aprilTagWebcam.update();
        if(aprilTagWebcam.getDetectedTags() == null) return;
        if (team == Team.BLUE) {
            findData(team);
            calculateBSide();
            calculateGamma();
            calculateAlpha();

            double firstTurnAngle = (-theta + gamma);
            turnPID((int) -firstTurnAngle);
            drivePID(-(bSide / 100), ((int) -firstTurnAngle));

            double secondAngleTurn = 180 - alpha;
            turnPID((int) secondAngleTurn);
        }
        aprilTagWebcam.close();
    }


    private void findData(Team team) {
        aprilTagWebcam.update();
        AprilTagDetection detection = aprilTagWebcam.getTagBySpecificId(team.getTeamAprilTagID());
        if (detection != null) {
            aSide = aprilTagWebcam.getAngle(detection, Rotation.RANGE);
            beta  = aprilTagWebcam.getAngle(detection, Rotation.YAW);
            theta = aprilTagWebcam.getAngle(detection, Rotation.BEARING);
        }
    }

    private void calculateBSide() {
        double thetaRad = Math.toRadians(theta);
        bSide = Math.sqrt(Math.pow(aSide, 2) + Math.pow(cSide, 2) - (2 * aSide * cSide * Math.cos(thetaRad)));
    }

    private void calculateGamma() {
        double cosGamma = (Math.pow(aSide, 2) + Math.pow(bSide, 2) - Math.pow(cSide, 2)) / (2 * aSide * bSide);
        cosGamma = Math.max(-1, Math.min(1, cosGamma));
        gamma = Math.toDegrees(Math.acos(cosGamma));
    }

    private void calculateAlpha() { alpha = 180 - (theta + gamma); }

    /**
     * Move the robot forward while using a pid/gyro correction
     * @param targetMeters distance to travel in meters
     * @param targetAngle angle to turn to in degrees*/

    public void drivePID(double targetMeters, int targetAngle) {
        double targetTicks = targetMeters * TICKS_PER_METER;
        double error = targetTicks;
        double lastError = 0;
        double integral = 0;

        resetOdometry();

        while (opModeIsActive() && Math.abs(error) > 50) {
                // FIXED: Removed the - sign that was causing infinite driving
            double currentPos = (leftOdo.getCurrentPosition() + rightOdo.getCurrentPosition()) / 2.0;
            error = targetTicks - currentPos;

            double derivative = error - lastError;
            integral += (Math.abs(error) < (0.1 * TICKS_PER_METER)) ? error : 0;

            double power = (Kp * (error / TICKS_PER_METER)) + (AutoEngineConfig.Ki * integral) + (AutoEngineConfig.Kd * derivative);

            // FIXED: Standardized steering logic to prevent the 90-degree spin
            double currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double steer = angleWrap(targetAngle - currentYaw) * AutoEngineConfig.STEER_P;

            power = Math.max(-0.7, Math.min(0.7, power));
            if (Math.abs(power) < MIN_POWER) power = Math.signum(power) * MIN_POWER;

            applyDrivePower(power, steer);
            lastError = error;
        }
        stopRobot();
    }

    /**
     * Move the robot sideways while using a pid/gyro correction
     * @param targetMeters distance to travel in meters
     * @param targetAngle angle to turn to in degrees*/

    public void strafePID(double targetMeters, int targetAngle) {
        double targetTicks = targetMeters * TICKS_PER_METER;
        double error = targetTicks;
        double lastError = 0;
        double integral = 0;

        resetOdometry();

        while (opModeIsActive() && Math.abs(error) > 50) {
            double currentPos = centerOdo.getCurrentPosition();
            error = targetTicks - currentPos;

            double derivative = error - lastError;
            integral += (Math.abs(error) < (0.1 * TICKS_PER_METER)) ? error : 0;

            double power = (AutoEngineConfig.strafe_Kp * (error / TICKS_PER_METER)) + (AutoEngineConfig.strafe_Ki * integral) + (AutoEngineConfig.strafe_Kd * derivative);

            // FIXED: Standardized steering to match drivePID
            double currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double steer = angleWrap(targetAngle - currentYaw) * AutoEngineConfig.STEER_P;

            power = Math.max(-0.7, Math.min(0.7, power));
            if (Math.abs(power) < MIN_POWER) power = Math.signum(power) * MIN_POWER;

            applyStrafePower(power, steer);
            lastError = error;
        }
        stopRobot();
    }

    /**
     * Turn the robot to desired angle
     * @param targetAngle angle to turn to in degrees*/

    public void turnPID(int targetAngle) {
        double currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double error = angleWrap(targetAngle - currentYaw);

        while (opModeIsActive() && Math.abs(error) > 1.0) {
            currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            error = angleWrap(targetAngle - currentYaw);
            double turnPower = error * 0.03;
            turnPower = Math.max(-0.6, Math.min(0.6, turnPower));
            if (Math.abs(turnPower) < 0.15) turnPower = Math.signum(turnPower) * 0.15;

            backLeft.setPower(-turnPower);
            frontLeft.setPower(-turnPower);
            backRight.setPower(turnPower);
            frontRight.setPower(turnPower);
        }
        stopRobot();
    }

    // --- HELPERS ---

    private void applyDrivePower(double p, double s) {
        backLeft.setPower(p - s);
        backRight.setPower(p + s);
        frontLeft.setPower(p - s);
        frontRight.setPower(p + s);
    }

    public void applyStrafePower(double strafe, double steer) {
        // FIXED: Re-mapped strafe powers to ensure lateral movement
        double fl = strafe + steer;
        double fr = -strafe - steer;
        double bl = -strafe + steer;
        double br = strafe - steer;

        double max = Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br))));
        if (max > 1.0) { fl /= max; fr /= max; bl /= max; br /= max; }

        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    private void stopRobot() {
        backLeft.setPower(0); backRight.setPower(0);
        frontLeft.setPower(0); frontRight.setPower(0);
        sleep(100);
    }

    private void resetOdometry() {
        leftOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        centerOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftOdo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightOdo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        centerOdo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void setMotorBehavior() {
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
    }

    private double angleWrap(double degrees) {
        while (degrees > 180) degrees -= 360;
        while (degrees < -180) degrees += 360;
        return degrees;
    }
}