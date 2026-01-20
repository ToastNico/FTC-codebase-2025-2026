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
        public static double strafe_Kp = 0;
        public static double strafe_Ki = 0;
        public static double strafe_Kd = 0;
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
    private double bSide = 0;
    private final double cSide = 150; // centi meter

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
        imu.resetYaw();
        waitForStart();
        imu.resetYaw();

        if (opModeIsActive()) {
            aprilTagWebcam.update();
            telemetry.update();
            runPath();
        }
    }

    // --- YOUR RESTORED LOGIC ---

    // Fixed: Now inside MovementEngine, so it can access drivePID and turnPID directly
    public void moveToShoot(Team team) {
        aprilTagWebcam.update();
        if(aprilTagWebcam.getDetectedTags() == null) return;
        if (team == Team.BLUE) {
            findData(team);

            // --- FIXED ORDER: Calculate side 'b' BEFORE angles gamma/alpha ---
            calculateBSide();   // Calculate distance to drive first
            calculateGamma();   // Calculate turn angle
            calculateAlpha();   // Calculate final alignment

            // First turn towards the target position
            double firstTurnAngle = (-theta + gamma);

            // Drive the distance (converted to meters)
           turnPID((int) -firstTurnAngle);


           drivePID(-(bSide / 100)/*convert to meters */, ((int) -firstTurnAngle));

            // Turn to the final shooting orientation
            double secondAngleTurn = 180 - alpha;
           turnPID((int) secondAngleTurn);

            telemetry.addData("turning Status", "completed");
            telemetry.addData("firstTurnAngle", firstTurnAngle);
            telemetry.addData("second angle", secondAngleTurn);
            telemetry.addData("Bearing", theta);
            telemetry.addData("Gamma", gamma);
            telemetry.addData("Alpha", alpha);
            telemetry.addData("Beta", beta);
            telemetry.addData("B side", bSide);
            telemetry.addData("A side", aSide);
            telemetry.addData("C side", cSide);

        }

        aprilTagWebcam.close();
    }

    void findData(Team team) {
        aprilTagWebcam.update();
        AprilTagDetection detection = aprilTagWebcam.getTagBySpecificId(team.getTeamAprilTagID());
        if (detection != null) {
            aSide = aprilTagWebcam.getAngle(detection, Rotation.RANGE);
            beta  = aprilTagWebcam.getAngle(detection, Rotation.YAW);
            theta = aprilTagWebcam.getAngle(detection, Rotation.BEARING);
        } else {
            // If it is null, we set values to 0 or a safe default
            aSide = 0;
            beta = 0;
            theta = 0;
        }
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
        double targetTicks = targetMeters * TICKS_PER_METER;
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

            if (Math.abs(error) < (0.1 * TICKS_PER_METER)) {
                integral = Math.max(-20, Math.min(20, integral));
            } else {
                integral = 0;
            }

            double power = (Kp * (error / TICKS_PER_METER))
                    + (AutoEngineConfig.Ki * integral)
                    + (AutoEngineConfig.Kd * derivative);

            double currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double steer = angleWrap(currentYaw - targetAngle) * -AutoEngineConfig.STEER_P;

            power = Math.max(-0.7, Math.min(0.7, power));
            if (Math.abs(power) < MIN_POWER)
                power = Math.signum(power) * MIN_POWER;

            applyDrivePower(power, steer);
            lastError = error;

            telemetry.addData("Error", error);
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
            double power = (Kp * (error / TICKS_PER_METER));

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




    // --- HELPERS ---

    private void applyDrivePower(double p, double s) {
        backLeft.setPower(p - s);
        backRight.setPower(p + s);
        frontLeft.setPower(p - s);
        frontRight.setPower(p + s);
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