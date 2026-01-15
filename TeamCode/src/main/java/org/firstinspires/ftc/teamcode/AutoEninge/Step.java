//package org.firstinspires.ftc.teamcode.AutoEninge;
//
//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
//
//public class Step {
//    private String name;
//    private double distance = 0;
//    private int angle = 0;
//
//    public Step(String name) { this.name = name; }
//
//    public Step forward(double meters) { this.distance = meters; return this; }
//    public Step backward(double meters) { this.distance = -meters; return this; }
//    public Step heading(int degrees) { this.angle = degrees; return this; }
//
//    /** Executes Turn-then-Drive (No strafing) */
//    public void run() {
//        telemetry.addLine("--- EXECUTING ---");
//        telemetry.addData("Task", name);
//        telemetry.update();
//
//        turnPID(angle);           // 1. Point the robot
//        drivePID(distance, angle); // 2. Drive straight
//    }
//
//    /** Executes a smooth curve */
//    public void runArc() {
//        telemetry.addLine("--- ARCING ---");
//        telemetry.addData("Task", name);
//        telemetry.update();
//        driveArc(distance, angle);
//    }
//
//    public Step builder(String name) {
//        return new Step(name);
//    }
//}
