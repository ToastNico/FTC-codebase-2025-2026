package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoEninge.AutoEngine;

@Autonomous(name="TuneTest")
public class AutoTest extends AutoEngine {

    @Override
    public void runPath() {
//        strafePID(3, 0);
//        strafePID(-1, 0);
//        drivePID(-1, 0);
//        drivePID(1, 0);

        arc(0.94, 0.5,
                t -> t
                .at(1.0, 90)
        );

        sleep(2000);
    }
}
