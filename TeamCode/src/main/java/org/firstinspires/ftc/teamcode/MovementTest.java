package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoEninge.MovementEngine;
import org.firstinspires.ftc.teamcode.AutoEninge.Team;

@Autonomous(name="TuneTest")
public class MovementTest extends MovementEngine {
    @Override
    public void runPath() {

        drivePID(3.5, 80);
        turnPID(95);
        moveToShoot(Team.BLUE);

        arc(4, 0.6, t -> t
                .at(0.3, 10));

    }
}
