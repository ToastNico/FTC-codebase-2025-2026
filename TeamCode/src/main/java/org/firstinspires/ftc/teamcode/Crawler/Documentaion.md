# Crawlier Framework: Precise Tuning & Pathing Guide

This document replaces the short overview with step-by-step, actionable instructions for calibrating three-wheel odometry, tuning the follower PID, building smooth waypoint paths, and integrating paths into TeleOp.

--- 
## Files & locations you will edit
- Localizer.java — hardware constants (TICKS_PER_MM, TRACK_WIDTH).  
  Path: TeamCode/src/main/java/org/firstinspires.ftc.teamcode/Crawler/Localizer.java
- TuningOpMode (provided) — run to tune PID values and to read telemetry.
- Path building code (where you call machine.start(...)) — wherever your TeleOp or autonomous code creates a path.

---

## 1) Physical odometry calibration (exact steps)

Purpose: make distances and headings reported by the Localizer match real-world movement.

A) Calibrate TICKS_PER_MM
1. Place the robot on a straight line and mark start.
2. Drive forward exactly D_actual = 1219.2 mm (use tape measure).
3. Record D_measured shown by your Localizer telemetry (D_meas).
4. Compute new constant:
   TICKS_PER_MM_new = TICKS_PER_MM_old * (D_meas / D_actual)
   Example: if telemetry says 1194.8 mm for a true 1219.2 mm,
   TICKS_PER_MM_new = old * (1194.8 / 1219.2).
5. Replace constant in Localizer.java, rebuild, and repeat until <2.5 mm error over 1219.2 mm.

B) Calibrate TRACK_WIDTH (rotation)
1. Spin robot in place a known amount: rotate exactly H_actual = 3600° (10 full turns) or 360° × N.
2. Record heading reported by Localizer (H_meas).
3. Compute:
   TRACK_WIDTH_new = TRACK_WIDTH_old * (H_meas / H_actual)
   If H_meas < H_actual, multiply TRACK_WIDTH_old by (H_meas / H_actual) — this reduces the track width.
4. Repeat for 1–3 full rotations to average out noise; target <1° cumulative error per full turn.

Notes:
- Always keep units consistent (millimeters, degrees).
- Use averages over multiple runs. Power rails and battery level affect wheel slip; calibrate with a charged battery.

---

## 2) PID tuning order and precise procedure (Follower controller)

Goal: tune in this strict order while running TuningOpMode.

Initial starting values (suggested):
- Kp = 0.02
- Kd = 0.0
- Kf = 0.0
- (These are starting points; your robot size & motors change magnitudes.)

Procedure:
1. Kp (Proportional)
   - Run a 609.6– 1219.2 mm straight move using the follower.
   - Increase Kp gradually (×1.5 steps) until the robot reaches targets reliably but without high-frequency oscillation.
   - If it oscillates or continuously hunts around the target, reduce Kp by ~20%.

2. Kd (Derivative)
   - With Kp set, increase Kd to reduce overshoot and to smooth settling.
   - Increase until approach is "damped" — the final approach should not overshoot and should settle in <0.2s.
   - If the robot becomes sluggish, reduce Kd.

3. Kf (Feedforward)
   - Use a small Kf to overcome motor friction and to improve steady-state accuracy on short moves.
   - Start at 0.001 and increase in 0.001 increments until small-position deadband (robot stuck ~5–12.7 mm) disappears.
   - Do not make Kf large; it should only supply the minimum constant drive.

Validation tests:
- 10 × 609.6 mm forward/back runs, record final error distribution (target error mean and std).
- 10 × 90° turns, record heading error.

Telemetry to monitor (display in TuningOpMode):
- target, pose error (x,y,heading), motor powers, Kp/Kd/Kf values, runtime.

---

## 3) Waypoints and smooth turning: exact usage

Method: waypoint(x, y, h, radius)
- x,y in millimeters relative to field origin used by your path builder.
- h is heading (degrees or radians per your framework — confirm Localizer uses degrees).
- radius (mm) is the tolerance sphere that, when entered, marks the waypoint as achieved.

Behavior:
- The follower will treat the waypoint as a soft attractor: it moves toward it but may blend into the next path segment once the radius is reached.
- radius small (≥0.8 mm): precise cornering, sharp turns.
- radius medium (101.6–304.8 mm): smooth arcs.
- radius large (≥457.2 mm): very wide, high-speed arcs.

Example path snippet (Java, inside your path builder):
```java
// Example usage in your path-building code
path.waypoint(609.6, 914.4, 90.0, 203.2); // move toward (609.6,914.4), finish when inside 203.2 mm, target heading 90 deg
path.waypoint(1219.2, 914.4, 90.0, 101.6); // tighter second corner
```

---

## 4) How to Use the Crawler Framework — Comprehensive Guide

### 4.1) Prerequisites & Hardware Setup

Before using Crawler, ensure:

1. **Three Encoder Dead Wheels** are mounted:
   - Left encoder wheel (attached to `frontLeft` motor)
   - Right encoder wheel (attached to `backLeft` motor)
   - Center strafe wheel (attached to `frontRight` motor)
   - All wheels should be same diameter (typical: 48mm Mecanum or 50mm Omni)

2. **Hardware Map Configuration** (in your `.xml` config):
   ```xml
   <Motor name="frontLeft" port="0" />   <!-- Left dead wheel -->
   <Motor name="backLeft" port="1" />    <!-- Right dead wheel -->
   <Motor name="frontRight" port="2" />  <!-- Center/Strafe dead wheel -->
   <Motor name="backRight" port="3" />   <!-- Drive motor (ignored by Localizer) -->
   <!-- Additional motors for your drive system -->
   ```

3. **Encoder Calibration Complete**:
   - TICKS_PER_MM and TRACK_WIDTH must be calibrated (see Section 1)
   - Without calibration, position tracking will be wildly inaccurate

---

### 4.2) Core Components Explained

#### **Localizer**
- **Purpose**: Continuously tracks robot's (x, y) position and heading using dead-wheel encoders
- **Inputs**: Raw encoder tick counts from three motors
- **Outputs**: x (mm), y (mm), heading (degrees)
- **Update cycle**: Call `localizer.update()` in every loop iteration
- **Key methods**:
  ```java
  localizer.update();              // Must call every cycle
  localizer.resetPose(x, y, heading); // Reset position (e.g., at start)
  localizer.getPoseX();            // Get X position in mm
  localizer.getPoseY();            // Get Y position in mm
  localizer.getHeading();          // Get heading in degrees
  ```

#### **Follower**
- **Purpose**: PID controller that drives the robot toward a target (x, y, heading)
- **Inputs**: Current pose (from Localizer), target pose (from tasks)
- **Outputs**: Motor power commands (sent to drive motors)
- **Tuning**: Kp, Kd, Kf constants (see Section 2)
- **Key methods**:
  ```java
  follower.setTarget(targetX, targetY, targetHeading);
  follower.update(currentX, currentY, currentHeading);
  double[] powers = follower.getPowers(); // [left, right, strafe]
  ```

#### **TaskBuilder**
- **Purpose**: Fluent API to build a sequence of waypoint tasks
- **Methods**:
  ```java
  .waypoint(x, y, heading, radius, speed)  // Add a soft waypoint
  .driveTo(x, y, heading, speed)           // Add a strict endpoint (small radius)
  .build()                                  // Return List<Task>
  ```
- **Parameters**:
  - `x, y`: Position in mm
  - `heading`: Target heading in degrees
  - `radius`: Tolerance zone (mm) — task completes when robot enters this circle
  - `speed`: Motor power multiplier (0.0–1.0)

#### **StateMachine**
- **Purpose**: Executes a sequence of tasks and tracks overall completion
- **Key methods**:
  ```java
  machine.start(List<Task> tasks);  // Begin execution
  machine.update();                  // Update current task (call every loop)
  boolean busy = machine.isBusy();   // Is any task still running?
  ```

---

### 4.3) Step-by-Step: Building Your First Autonomous Path

#### **Example 1: Simple Forward-and-Stop Movement**

```java
@Autonomous(name = "Simple Forward", group = "Crawler")
public class SimpleForwardAuto extends LinearOpMode {
    
    @Override
    public void runOpMode() throws InterruptedException {
        // Step 1: Initialize hardware
        DcMotor leftMotor = hardwareMap.get(DcMotor.class, "backRight");
        DcMotor rightMotor = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor strafeMotor = hardwareMap.get(DcMotor.class, "frontRight");
        
        Localizer localizer = new Localizer(hardwareMap);
        Follower follower = new Follower(new Robot(hardwareMap)); // or your drive class
        
        // Step 2: Create task sequence
        TaskBuilder builder = new TaskBuilder(follower, localizer);
        List<Task> path = builder
            .waypoint(1219.2, 0, 0, 203.2, 0.7)  // Move 1219.2mm forward at 70% speed
            .build();
        
        // Step 3: Wait for start
        telemetry.addLine("Ready: Simple Forward Path");
        telemetry.update();
        waitForStart();
        
        // Step 4: Reset position to origin
        localizer.resetPose(0, 0, 0);
        
        // Step 5: Start path execution
        StateMachine machine = new StateMachine();
        machine.start(path);
        
        // Step 6: Main loop — update localizer and machine
        while (opModeIsActive() && machine.isBusy()) {
            localizer.update();      // Refresh position from encoders
            machine.update();        // Update path follower
            
            // Send motor powers
            double[] powers = follower.getPowers();
            leftMotor.setPower(powers[0]);
            rightMotor.setPower(powers[1]);
            strafeMotor.setPower(powers[2]);
            
            // Display diagnostics
            telemetry.addData("X (mm)", "%.2f", localizer.getPoseX());
            telemetry.addData("Y (mm)", "%.2f", localizer.getPoseY());
            telemetry.addData("Heading (deg)", "%.2f", localizer.getHeading());
            telemetry.update();
        }
        
        // Step 7: Stop motors
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        strafeMotor.setPower(0);
        
        telemetry.addLine("Path complete!");
        telemetry.update();
    }
}
```

---

#### **Example 2: Multi-Waypoint S-Curve Path**

```java
// Create a smooth S-curve through three points
List<Task> sCurvePath = builder
    .waypoint(609.6, 0, 0, 152.4, 0.8)        // First leg: move 609.6mm forward
    .waypoint(609.6, 609.6, 90, 152.4, 0.8)   // Turn right 90° while moving forward
    .waypoint(1219.2, 609.6, 180, 152.4, 0.8) // Turn left toward endpoint
    .driveTo(1219.2, 609.6, 180, 0.5)         // Final tight approach at 50% speed
    .build();
```

**What happens:**
1. Robot starts at (0,0) heading 0°
2. Moves to waypoint 1: (609.6, 0) — travels straight forward
3. Enters tolerance radius; begins blending to waypoint 2
4. Moves to waypoint 2: (609.6, 609.6) heading 90° — smooth arc turn
5. Moves to waypoint 3: (1219.2, 609.6) heading 180° — another arc
6. Final driveTo ensures tight endpoint accuracy at reduced speed

---

#### **Example 3: Precise Corner Movement (Small Radius)**

```java
// Use small radius for sharp, precise corners
List<Task> sharpCorners = builder
    .waypoint(609.6, 0, 0, 50.8, 0.6)        // Small radius = tight corner
    .waypoint(609.6, 609.6, 90, 50.8, 0.6)   // Sharp 90° turn
    .driveTo(1219.2, 609.6, 180, 0.5)        // Precise endpoint
    .build();
```

---

#### **Example 4: High-Speed Smooth Arcs (Large Radius)**

```java
// Use large radius for smooth, high-speed movement
List<Task> fastPath = builder
    .waypoint(609.6, 304.8, 45, 304.8, 1.0)   // Large radius + full speed = smooth, fast
    .waypoint(914.4, 609.6, 90, 304.8, 1.0)   // Sweeping arc
    .driveTo(1219.2, 609.6, 0, 0.8)            // Fast approach
    .build();
```

---

### 4.4) Integrating Paths into Autonomous Modes

#### **Pattern 1: Sequential Paths**
```java
StateMachine machine1 = new StateMachine();
machine1.start(pathA);

while (opModeIsActive() && machine1.isBusy()) {
    localizer.update();
    machine1.update();
    // ... update motors, telemetry
}

// Once pathA complete, start pathB
StateMachine machine2 = new StateMachine();
machine2.start(pathB);

while (opModeIsActive() && machine2.isBusy()) {
    localizer.update();
    machine2.update();
    // ... update motors, telemetry
}
```

#### **Pattern 2: Conditional Branching**
```java
if (rightSideChosen) {
    machine.start(rightPath);
} else {
    machine.start(leftPath);
}

while (opModeIsActive() && machine.isBusy()) {
    localizer.update();
    machine.update();
    // ...
}

// After path, perform other actions (intake, outtake, etc.)
performIntake();
performOuttake();
```

---

### 4.5) Integrating Paths into TeleOp Mode

#### **Pattern 1: Press Button to Run Path**
```java
@TeleOp(name = "TeleOp with Paths", group = "Crawler")
public class TeleOpWithPaths extends LinearOpMode {
    private StateMachine machine = null;
    private boolean pathActive = false;
    
    @Override
    public void runOpMode() throws InterruptedException {
        Localizer localizer = new Localizer(hardwareMap);
        Follower follower = new Follower(new Robot(hardwareMap));
        DcMotor leftMotor = hardwareMap.get(DcMotor.class, "backRight");
        DcMotor rightMotor = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor strafeMotor = hardwareMap.get(DcMotor.class, "frontRight");
        
        localizer.resetPose(0, 0, 0);
        waitForStart();
        
        while (opModeIsActive()) {
            localizer.update();
            
            // If 'A' pressed and no path running, start a path
            if (gamepad1.a && !pathActive) {
                TaskBuilder builder = new TaskBuilder(follower, localizer);
                List<Task> quickPath = builder
                    .waypoint(609.6, 0, 0, 101.6, 0.7)
                    .build();
                
                machine = new StateMachine();
                machine.start(quickPath);
                pathActive = true;
            }
            
            // Check if path is still running
            if (pathActive) {
                machine.update();
                if (!machine.isBusy()) {
                    pathActive = false;
                }
            }
            
            // Get motor powers from follower or manual controls
            double[] powers;
            if (pathActive) {
                powers = follower.getPowers();  // Use path follower
            } else {
                // Manual control
                double drive = -gamepad1.left_stick_y;
                double strafe = gamepad1.left_stick_x;
                double rotate = gamepad1.right_stick_x;
                powers = new double[] {drive - rotate, drive + rotate, strafe};
            }
            
            leftMotor.setPower(powers[0]);
            rightMotor.setPower(powers[1]);
            strafeMotor.setPower(powers[2]);
            
            telemetry.addData("X (mm)", "%.2f", localizer.getPoseX());
            telemetry.addData("Y (mm)", "%.2f", localizer.getPoseY());
            telemetry.addData("Path Active", pathActive);
            telemetry.update();
        }
    }
}
```

---

### 4.6) Understanding Waypoint Parameters

#### **radius Parameter (Tolerance Zone)**

The radius defines how "loose" or "tight" the waypoint is:

| Radius (mm) | Behavior | Best For |
|-------------|----------|----------|
| 25.4–50.8 (1–2") | Sharp, tight corners | Precise scoring, obstacle gaps |
| 101.6–152.4 (4–6") | Medium blend | General waypoints, balanced |
| 203.2–304.8 (8–12") | Smooth arcs | High-speed movement, curves |
| 457.2+ (18"+) | Very wide arcs | Sweep turns, minimal slowdown |

**Example comparison:**
```java
// Tight precision
.waypoint(609.6, 609.6, 90, 25.4, 0.5)

// Medium (balanced)
.waypoint(609.6, 609.6, 90, 152.4, 0.7)

// Smooth arc
.waypoint(609.6, 609.6, 90, 304.8, 0.9)
```

#### **speed Parameter (Motor Power)**

Speed scales the motor power output (0.0 = stopped, 1.0 = full power):

```java
.waypoint(x, y, h, radius, 0.3)   // Slow: 30% power (very careful)
.waypoint(x, y, h, radius, 0.6)   // Medium: 60% power (balanced)
.waypoint(x, y, h, radius, 0.9)   // Fast: 90% power (aggressive)
```

**Note:** For stability and smooth turning, do not exceed 0.8 during normal operation. Higher speeds risk oscillation and encoder slippage.

---

### 4.7) Debugging & Telemetry

#### **Essential Telemetry for Path Debugging**

```java
while (opModeIsActive() && machine.isBusy()) {
    localizer.update();
    machine.update();
    
    // Pose information
    telemetry.addData("X (mm)", "%.1f", localizer.getPoseX());
    telemetry.addData("Y (mm)", "%.1f", localizer.getPoseY());
    telemetry.addData("Heading (deg)", "%.1f", localizer.getHeading());
    
    // Target information (if available from Follower)
    telemetry.addData("Target X", "%.1f", follower.getTargetX());
    telemetry.addData("Target Y", "%.1f", follower.getTargetY());
    
    // Error metrics
    double errorX = follower.getTargetX() - localizer.getPoseX();
    double errorY = follower.getTargetY() - localizer.getPoseY();
    telemetry.addData("Error X (mm)", "%.1f", errorX);
    telemetry.addData("Error Y (mm)", "%.1f", errorY);
    
    // Motor powers
    double[] powers = follower.getPowers();
    telemetry.addData("Left Power", "%.3f", powers[0]);
    telemetry.addData("Right Power", "%.3f", powers[1]);
    telemetry.addData("Strafe Power", "%.3f", powers[2]);
    
    telemetry.update();
}
```

#### **Common Issues & Fixes**

| Problem | Likely Cause | Fix |
|---------|--------------|-----|
| Robot drifts left/right | TRACK_WIDTH miscalibrated | Re-calibrate rotation (Section 1B) |
| Robot overshoots waypoints | Kp too high | Reduce Kp by 10–20% (Section 2) |
| Robot oscillates side-to-side | Kd too low or Kp too high | Increase Kd or reduce Kp |
| Robot moves but heading wrong | CENTER_OFFSET incorrect | Check dead wheel alignment |
| Path stops mid-execution | Encoder failure or motor stalled | Check encoder connections; verify motor power |
| Position jumps erratically | Encoder noise or slipping | Verify encoder mounting; check wheel traction |

---

### 4.8) Best Practices

1. **Always Calibrate First**
   - Do not attempt paths until TICKS_PER_MM and TRACK_WIDTH are within 5% accuracy
   - Recalibrate if you swap encoders or robots

2. **Start with Manual Control**
   - Before building complex paths, test simple movements manually
   - Verify motor directions and encoder polarity

3. **Use Moderate Speeds**
   - Keep speed ≤ 0.7 for initial testing
   - Increase gradually after behavior is stable

4. **Test Incrementally**
   - Test each waypoint separately before chaining
   - Add one waypoint at a time; verify before extending

5. **Monitor Telemetry**
   - Always display real-time position and error in TeleOp for live tuning
   - Save telemetry logs to analyze accuracy post-run

6. **Reset Pose Appropriately**
   - Call `resetPose(0, 0, 0)` at start of each path
   - Use `resetPose(lastX, lastY, lastHeading)` to chain multi-segment paths

7. **Account for Battery Voltage**
   - Calibration assumes full battery charge
   - As voltage drops, wheel slip increases; paths become less accurate
   - Recommend re-calibrating with battery at ~50% charge as well

---

## 5) Strengths vs Weaknesses

### Strengths ✓

1. **Precision & Repeatability**
   - Dead-wheel odometry provides accurate position tracking without relying on wheel slippage
   - PID-based follower ensures consistent path tracking with tunable accuracy
   - Small tolerance radius enables tight corner navigation

2. **Smooth, Elegant Movement**
   - Waypoint-based system with soft attractors creates smooth, curved paths
   - No sharp turns or sudden direction changes; robot naturally blends between waypoints
   - Scales speed based on distance to target (inherent feedforward behavior)

3. **Easy Integration**
   - TaskBuilder API simplifies path creation (fluent interface)
   - Works seamlessly with both TeleOp and Autonomous modes
   - Telemetry built-in for real-time debugging and tuning

4. **Flexible Waypoint Control**
   - Adjust radius per waypoint for dynamic speed/precision tradeoffs
   - Support for heading-locked movement and free-heading arcs
   - Speed limiting per waypoint (0.0–1.0 multiplier)

5. **Robust Calibration Process**
   - Step-by-step calibration procedures minimize field-specific tuning
   - Supports iterative refinement with clear error metrics
   - Metric system for international standardization

### Weaknesses ✗

1. **Requires Initial Calibration**
   - TICKS_PER_MM and TRACK_WIDTH must be calibrated before reliable operation
   - Wheel wear, battery voltage changes, and field surface variations affect accuracy
   - Re-calibration needed if encoders are replaced or robot geometry changes

2. **Sensitive to Encoder Errors**
   - Dead-wheel odometry drift accumulates over long distances (no external reference)
   - Single encoder failure causes loss of one degree of freedom
   - Encoder mounting misalignment directly impacts heading accuracy

3. **PID Tuning Complexity**
   - Requires manual Kp/Kd/Kf tuning; no auto-tuning
   - Different robot sizes/motors need different starting constants
   - Poor tuning can cause oscillation, sluggish response, or overshooting

4. **Limited by Encoder Resolution**
   - Precision capped by encoder resolution and motor gearing
   - Very small movements (< 25.4 mm) may not be reliably detected
   - Quantization error increases on short distances

5. **No Obstacle Avoidance**
   - Framework assumes clear paths; will collide if waypoint blocked
   - No feedback from sensors (lidar, sonar, bumpers) to abort on collision
   - Useful for controlled environments; not suitable for dynamic, unpredictable fields

6. **Robot Orientation Dependent**
   - Relies on accurate initial heading; wrong starting angle propagates through path
   - Heading drift from rotational slippage can accumulate
   - Mecanum/strafe limitations depend on field friction and motor tuning

### When to Use Crawler

**Best for:**
- Autonomous routines with known, repeatable paths
- Precision scoring tasks requiring tight positional accuracy
- Smooth, visually impressive movement
- Pre-match environments with consistent field conditions

**Not ideal for:**
- Real-time obstacle avoidance / dynamic environments
- First-time teams without encoder/PID experience
- Fields with significant surface variation or slipperiness
- Tasks requiring sub-mm precision (physical limitations of typical FTC motors)

### Performance Guidelines

- **Straight line accuracy**: ±25.4 mm over 1219.2 mm (after calibration)
- **Heading accuracy**: ±2–5° over full rotation (depends on TRACK_WIDTH calibration)
- **Path following speed**: Up to 80% motor power recommended for stability
- **Typical calibration time**: 20–30 minutes (5–10 iterations per constant)