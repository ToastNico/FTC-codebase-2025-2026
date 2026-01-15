package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoEninge.AutoEngine;

@Autonomous(name="TuneTest")
public class AutoTest extends AutoEngine {
    @Override
    public void runPath() {
//        drivePID(0.25,0);
            vectorMove(1, 0);
    }
}
