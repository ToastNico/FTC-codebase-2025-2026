package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoEninge.MovementEngine;
import org.firstinspires.ftc.teamcode.AutoEninge.Team;

@Autonomous(name="TuneTest")
public class MovementTest extends MovementEngine {
    @Override
    public void runPath() {

        // Generated Path (robot-oriented)
        // Start: x=0.00 y=0.00 h=0.0deg

        turnPID(0); // face start heading
        turnPID(-146);
        drivePID(1.23, -146);
        turnPID(-128); // waypoint heading
        turnPID(36);
        drivePID(2.53, 36);
        turnPID(91); // waypoint heading


    }
}
