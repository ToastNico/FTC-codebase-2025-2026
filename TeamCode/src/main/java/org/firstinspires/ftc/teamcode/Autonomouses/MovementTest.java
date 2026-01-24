package org.firstinspires.ftc.teamcode.Autonomouses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoEninge.MovementEngine;
import org.firstinspires.ftc.teamcode.AutoEninge.Team;
import org.firstinspires.ftc.teamcode.AutoEninge.Robot;

@Autonomous(name="BlueAuto")
public class MovementTest extends MovementEngine {
    @Override
    public void runPath() throws InterruptedException {
// Generated Path (robot-oriented)
// Start: x=-1.76 y=-0.01 h=0.0deg

        turnPID(0); // face start heading
        turnPID(-10);
        drivePID(1.29, -10);
        turnPID(46); // waypoint heading
        robot.shootSequence();
        turnPID(35);
        drivePID(1.88, 35);
        turnPID(-91); // waypoint heading


    }
}
