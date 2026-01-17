package org.firstinspires.ftc.teamcode.AutoEninge;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Vision.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.Vision.Rotation;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public abstract class AutoEngine extends LinearOpMode {

    // --- CONFIGURATION ---
    @Config
    static class AutoEngineConfig {
        public static final double ODO_WHEEL_DIAMETER_METERS = 0.048;
        public static final double ENCODER_TICKS_PER_REV = 2000;
        public static final double ODO_WHEEL_CIRCUMFERENCE = ODO_WHEEL_DIAMETER_METERS * Math.PI;
        public static final double TICKS_PER_METER = (ENCODER_TICKS_PER_REV / ODO_WHEEL_CIRCUMFERENCE);

        public static double Kp = 0.6;
        public static double Kd = 0;
        public static double Ki = 0.0015;
        public static final double STEER_P = 0.02;
        public static final double MIN_POWER = 0.2;
    }

    // --- HARDWARE ---
    protected DcMotor backLeft, backRight, frontLeft, frontRight;
    protected DcMotor leftOdo, rightOdo, centerOdo;
    protected IMU imu;

    // The single, shared webcam instance
    protected AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();

    // --- SHOOTING MATH VARIABLES ---
    // Defined here so all methods in the class can see them
    private double alpha = 0;
    private double beta = 0;
    private double gamma = 0;
    private double theta = 0;

    private double aSide = 0;
    // Initialized to 1.0 to prevent divide-by-zero errors in the first calculation step
    private double bSide = 1.0;
    private final double cSide = 1500; // millimeters

    public abstract void runPath();

    @Override
    public void runOpMode() throws InterruptedException {
        // 1. Init Hardware
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

        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart();
        imu.resetYaw();

        if (opModeIsActive()) {
            aprilTagWebcam.update();
            runPath();
        }
    }

    // --- YOUR RESTORED LOGIC ---

    // Fixed: Now inside AutoEngine, so it can access drivePID and turnPID directly
    public void moveToShoot(Team team) {

        if(aprilTagWebcam.getDetectedTags() == null) return;
        if (team == Team.BLUE) {
            findData(team);

            // --- FIXED ORDER: Calculate side 'b' BEFORE angles gamma/alpha ---
            calculateBSide();   // Calculate distance to drive first
            calculateGamma();   // Calculate turn angle
            calculateAlpha();   // Calculate final alignment

            // First turn towards the target position
            double firstTurnAngle = (theta + gamma) ;

            // Drive the distance (converted to meters)
//            turnPID((int) firstTurnAngle);
            drivePID((bSide / 1000.0) * -1, ((int) firstTurnAngle * -1));

            // Turn to the final shooting orientation
            double secondAngleTurn = 180 - alpha;
            turnPID((int) secondAngleTurn);
        }

        aprilTagWebcam.close();
    }

    void findData(Team team) {
        aSide = aprilTagWebcam.getAngle(aprilTagWebcam.getTagBySpecificId(team.getTeamAprilTagID()), Rotation.RANGE);
        beta = aprilTagWebcam.getAngle(aprilTagWebcam.getTagBySpecificId(team.getTeamAprilTagID()), Rotation.YAW);
        theta = aprilTagWebcam.getAngle(aprilTagWebcam.getTagBySpecificId(team.getTeamAprilTagID()), Rotation.BEARING);
    }

    /**
     * Step 1: Calculate the distance we need to drive (bSide)
     * using the Law of Cosines (SAS: Side-Angle-Side)
     */
    private void calculateBSide() {
        double thetaRad = Math.toRadians(theta);
        // Formula: b = sqrt(a^2 + c^2 - 2ac * cos(theta))
        bSide = Math.sqrt(
                Math.pow(aSide, 2) + Math.pow(cSide, 2) - (2 * aSide * cSide * Math.cos(thetaRad))
        );
    }

    /**
     * Step 2: Calculate Gamma (The angle offset for the drive path)
     * using the Law of Cosines (SSS: Side-Side-Side)
     */
    private void calculateGamma() {
        // Formula: gamma = arccos( (a^2 + b^2 - c^2) / 2ab )
        double cosGamma = (Math.pow(aSide, 2) + Math.pow(bSide, 2) - Math.pow(cSide, 2)) / (2 * aSide * bSide);

        // Clamp value between -1 and 1 to prevent Math errors
        cosGamma = Math.max(-1, Math.min(1, cosGamma));

        // Math.acos returns the inverse cosine in Radians
        gamma = Math.toDegrees(Math.acos(cosGamma));
    }

    /**
     * Step 3: Calculate Alpha (Final interior angle)
     */
    private void calculateAlpha() {
        // Standard triangle rule: All interior angles = 180
        alpha = 180 - (theta + gamma);
    }

    // --- DRIVING METHODS ---

    public void drivePID(double targetMeters, int targetAngle) {
        double targetTicks = targetMeters * AutoEngineConfig.TICKS_PER_METER;
        double error = targetTicks;
        double lastError = 0;
        double integral = 0;

        resetOdometry();

        // Added simple timeout protection or OpMode check
        while (opModeIsActive() && Math.abs(error) > 50) {
            double currentPos = (leftOdo.getCurrentPosition() + rightOdo.getCurrentPosition()) / 2.0;
            currentPos = -currentPos; // Adjust based on wiring

            error = targetTicks - currentPos;

            double derivative = error - lastError;
            integral += error;

            if (Math.abs(error) < (0.1 * AutoEngineConfig.TICKS_PER_METER)) {
                integral = Math.max(-20, Math.min(20, integral));
            } else {
                integral = 0;
            }

            double power = (AutoEngineConfig.Kp * (error / AutoEngineConfig.TICKS_PER_METER))
                    + (AutoEngineConfig.Ki * integral)
                    + (AutoEngineConfig.Kd * derivative);

            double currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double steer = angleWrap(currentYaw - targetAngle) * -AutoEngineConfig.STEER_P;

            power = Math.max(-0.7, Math.min(0.7, power));
            if (Math.abs(power) < AutoEngineConfig.MIN_POWER)
                power = Math.signum(power) * AutoEngineConfig.MIN_POWER;

            applyDrivePower(power, steer);
            lastError = error;

            telemetry.addData("Error", error);
            telemetry.update();
        }
        stopRobot();
    }

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

    private void stopRobot() {
        backLeft.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        sleep(150);
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

        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
    }

    private double angleWrap(double degrees) {
        while (degrees > 180) degrees -= 360;
        while (degrees < -180) degrees += 360;
        return degrees;
    }
}