package org.firstinspires.ftc.teamcode.AutoEninge;

import static org.firstinspires.ftc.teamcode.AutoEninge.AutoEngine.AutoEngineConfig.MIN_POWER;
import static org.firstinspires.ftc.teamcode.AutoEninge.AutoEngine.AutoEngineConfig.TICKS_PER_METER;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Vision.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.Vision.Rotation;

public abstract class AutoEngine extends LinearOpMode {

    // --- TUNING VARIABLES (Adjust these for your robot) ---
    // Calculate: (Ticks per rotation of encoder) / (Circumference of deadwheel)
    // --- ODOMETRY HARDWARE CONSTANTS ---
// goBILDA 48mm Pod: https://www.gobilda.com/swingarm-odometry-pod-48mm-wheel/

    @Config
    static class AutoEngineConfig {
        public static final double ODO_WHEEL_DIAMETER_METERS = 0.048;//48 mm
        public static final double ENCODER_TICKS_PER_REV = 2000; // PPR for SRE Magnetic Encoder
        //correction factor
        public static final double DISTANCE_CORRECTION = 1.0204;

        // --- AUTO-CALCULATED CONSTANTS ---
        public static final double ODO_WHEEL_CIRCUMFERENCE = ODO_WHEEL_DIAMETER_METERS * Math.PI;
        public static final double TICKS_PER_METER = (ENCODER_TICKS_PER_REV / ODO_WHEEL_CIRCUMFERENCE);


        // PID Coefficients
        public static double Kp = 0.6;  // Power: how fast it moves toward target
        public static double Kd = 0; // Dampening: prevents shaking/overshoot
        public static double Ki = 0.0015; // Integral: handles friction at the very end

        public static double strafe_Kp = 0.8;
        public static double strafe_Kd = 0;
        public static double strafe_Ki = 0.0015;

        public static final double STEER_P = 0.02; // How aggressively it fixes its angle
        public static final double MIN_POWER = 0.2; // Minimum power to overcome friction
    }

    // --- HARDWARE ---
    protected DcMotor backLeft, backRight, frontLeft, frontRight;
    protected DcMotor leftOdo, rightOdo, centerOdo; // Deadwheels
    protected IMU imu;
    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();


    public abstract void runPath();

    @Override
    public void runOpMode() throws InterruptedException {
        aprilTagWebcam.init(hardwareMap, telemetry);
        // 1. Drivetrain Hardware
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");

        leftOdo = hardwareMap.get(DcMotor.class, "frontLeft");
        rightOdo = hardwareMap.get(DcMotor.class, "backRight");
        centerOdo = hardwareMap.get(DcMotor.class, "backLeft");

        setMotorBehavior();

        // 3. IMU Initialization
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));

        resetOdometry();

        telemetry.addData("Status", "Odometry Engine Ready");
        telemetry.update();

        waitForStart();
        imu.resetYaw();

        if (opModeIsActive()) {
            aprilTagWebcam.update();
            runPath();
        }
    }

    // --- PID MOVEMENT COMMANDS ---

    /**
     * Move straight (Forward/Backward) using PID and Deadwheels
     */
    public void drivePID(double targetMeters, int targetAngle) {
        double targetTicks = targetMeters * TICKS_PER_METER;
        double error = targetTicks;
        double lastError = 0;
        double integral = 0;

        final int maxError = 50;

        resetOdometry();

        while (opModeIsActive() && Math.abs(error) > maxError) {
            double currentPos = (leftOdo.getCurrentPosition() + rightOdo.getCurrentPosition()) / 2.0; //Devide by two is for the average between the two odometry pods

            currentPos = currentPos * -1; // if the odometry pods are mounted backwards

            error = targetTicks - currentPos;

            // PID Logic
            double derivative = error - lastError;
            integral += error;

            // Anti-windup cap
            if (Math.abs(error) < (0.1 * TICKS_PER_METER)) { // Only use Integral when close
                integral = Math.max(-20, Math.min(20, integral));
            } else {
                integral = 0;
            }

            double power = (AutoEngineConfig.Kp * (error / TICKS_PER_METER)) + (AutoEngineConfig.Ki * integral) + (AutoEngineConfig.Kd * derivative);

            // Steering with Angle Wrap
            double currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double steer = angleWrap(currentYaw - targetAngle) * -AutoEngineConfig.STEER_P;

            power = Math.max(-0.7, Math.min(0.7, power));
            if (Math.abs(power) < MIN_POWER) power = Math.signum(power) * MIN_POWER;

            applyDrivePower(power, steer);
            lastError = error;

            telemetry.addData("Target Ticks", targetTicks);
            telemetry.addData("Current Pos", currentPos);
            telemetry.addData("Error", error);
            telemetry.update();

        }
        stopRobot();
    }

    /**
     * Strafe (Left/Right) using Center Deadwheel
     */
    /**
     * Rotates the robot in place to a specific heading.
     */
    public void turnPID(int targetAngle) {
        // 1. Calculate error
        double currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double error = angleWrap(targetAngle - currentYaw);

        // 2. Loop until error is small (e.g., < 1 degree)
        while (opModeIsActive() && Math.abs(error) > 1.0) {
            currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            error = angleWrap(targetAngle - currentYaw);

            // Simple P-Control for turning
            // We use a higher P value here because turning needs more punch than steering correction
            double turnPower = error * 0.03;

            // Clamp power to avoid moving too fast or stalling
            turnPower = Math.max(-0.6, Math.min(0.6, turnPower));
            if (Math.abs(turnPower) < 0.15) turnPower = Math.signum(turnPower) * 0.15;

            // Apply power (Turn Right = Left Forward, Right Back)
            // Note: Check your motor directions!
            backLeft.setPower(-turnPower);
            frontLeft.setPower(-turnPower);
            backRight.setPower(turnPower);
            frontRight.setPower(turnPower);

            telemetry.addData("Target", targetAngle);
            telemetry.addData("Heading", currentYaw);
            telemetry.update();
        }
        stopRobot();
    }

    public void strafePID(double targetMeters, int targetAngle) {
        double targetTicks = targetMeters * TICKS_PER_METER;
        double error = targetTicks;
        double lastError = 0;
        double integral = 0;

        final int maxError = 50;

        resetOdometry();

        while (opModeIsActive() && Math.abs(error) > maxError) {
            double currentPos = centerOdo.getCurrentPosition();

            //currentPos = currentPos * -1; // if the odometry pods are mounted backwards

            error = targetTicks - currentPos;

            // PID Logic
            double derivative = error - lastError;
            integral += error;

            // Anti-windup cap
            if (Math.abs(error) < (0.1 * TICKS_PER_METER)) { // Only use Integral when close
                integral = Math.max(-20, Math.min(20, integral));
            } else {
                integral = 0;
            }

            double power = (AutoEngineConfig.strafe_Kp * (error / TICKS_PER_METER)) + (AutoEngineConfig.strafe_Ki * integral) + (AutoEngineConfig.strafe_Kd * derivative);

            // Steering with Angle Wrap
            double currentYaw = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double steer = angleWrap(currentYaw - targetAngle) * -AutoEngineConfig.STEER_P;

            power = Math.max(-0.7, Math.min(0.7, power));
            if (Math.abs(power) < MIN_POWER) power = Math.signum(power) * MIN_POWER;

            applyStrafePower(power, steer);
            lastError = error;

            telemetry.addData("Target Ticks", targetTicks);
            telemetry.addData("Current Pos", currentPos);
            telemetry.addData("Error", error);
            telemetry.update();

        }
        stopRobot();
    }

    /**
     * Advanced Arc with Keyframes
     *
     * @param meters   Total distance
     * @param maxPower Speed cap
     * @param animator The animation instructions
     */
    public void arc(double meters, double maxPower, AnimationBuilder animator) {
        double targetTicks = meters * TICKS_PER_METER;
        double startHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        // 1. Setup the Timeline
        HeadingTimeline timeline = new HeadingTimeline();
        animator.build(timeline); // Execute the user's instructions

        resetOdometry();

        while (opModeIsActive()) {
            double currentPos = (leftOdo.getCurrentPosition() + rightOdo.getCurrentPosition()) / 2.0;

            // Calculate Progress (0.0 to 1.0)
            double progress = Math.abs(currentPos / targetTicks);
            if (progress >= 1.0) break;

            // 2. ASK THE TIMELINE FOR HEADING
            double targetHeading = timeline.getTarget(progress, startHeading);

            // Standard Drive Logic
            double error = Math.abs(targetTicks) - Math.abs(currentPos);
            double power = (AutoEngineConfig.Kp * (error / TICKS_PER_METER));

            power = Math.max(-maxPower, Math.min(maxPower, power));
            if (Math.abs(power) < MIN_POWER) power = Math.signum(power) * MIN_POWER;

            double currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double steer = angleWrap(currentYaw - targetHeading) * -AutoEngineConfig.STEER_P;

            applyDrivePower(power, steer);

            // Telemetry for debugging
            telemetry.addData("Progress", "%.2f", progress);
            telemetry.addData("Target Head", "%.1f", targetHeading);
            telemetry.update();
        }
        stopRobot();
    }

    /* */

    // --- HELPERS ---

    private void applyDrivePower(double p, double s) {
        backLeft.setPower(p - s);
        backRight.setPower(p + s);  // Changed from -p to p
        frontLeft.setPower(p - s);  // Ensure this matches your wiring
        frontRight.setPower(p + s);
    }

    private void stopRobot() {
        backLeft.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        sleep(150); // Small pause for stability
    }

    public void applyStrafePower(double strafe, double steer) {
        // Mecanum Strafe Pattern:
        // FrontLeft and BackRight go one way
        // FrontRight and BackLeft go the other way
        double fl = strafe + steer;
        double fr = -strafe - steer;
        double bl = -strafe + steer;
        double br = strafe - steer;

        // Normalize power so no motor exceeds 1.0
        double max = Math.max(Math.abs(fl), Math.max(Math.abs(fr),
                Math.max(Math.abs(bl), Math.abs(br))));
        if (max > 1.0) {
            fl /= max;
            fr /= max;
            bl /= max;
            br /= max;
        }

        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
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


    class ShootAlignment {
        //First move to the rough, position of the shooting thing
        //Then align more accurately

        //TODO remember to convert to 2D

        double alpha = 0;
        double beta = 0;
        double gamma = 0;
        double theta = 0;

        double aSide = 0;
        double bSide = 0;
        final double cSide = 1500; //milli meter
        Robot robot;

        public ShootAlignment(HardwareMap hwMap) {
            robot = new Robot(hwMap);
        }

        AprilTagWebcam webcam = new AprilTagWebcam();


        public void moveToShoot(Team team) {

            if (team == Team.BLUE) {
                findData(team);
                calculateGamma();
                calculateBeta();
                calculateAlpha();
                //First turn towards the april tag
                double firstTurnAngle = theta + gamma;
                //turnPID((int) firstTurnAngle); Add this if it does not work
                drivePID(bSide / 1000, (int) firstTurnAngle);

                double secondAngleTurn = 180 - alpha;
                turnPID((int) secondAngleTurn);

            }
        }

        void findData(Team team) {
            aSide = webcam.getAngle(webcam.getTagBySpecificId(team.getTeamAprilTagID()), Rotation.RANGE);
            beta = webcam.getAngle(webcam.getTagBySpecificId(team.getTeamAprilTagID()), Rotation.YAW);
            theta = webcam.getAngle(webcam.getTagBySpecificId(team.getTeamAprilTagID()), Rotation.BEARING);
        }

        private void calculateGamma() {
            gamma = Math.pow(Math.cos((Math.pow(aSide, 2) + Math.pow(bSide, 2) - Math.pow(cSide, 2)) / (2 * aSide * bSide)), -1);
        }

        private void calculateBeta() {
            bSide = alpha * Math.cos(gamma) + Math.sqrt(
                    Math.pow(gamma, 2) + Math.pow(beta, 2) * Math.pow(Math.sin(gamma), 2)
            );
        }

        private void calculateAlpha() {
            alpha = 180 - (beta + gamma);
        }
    }
}


