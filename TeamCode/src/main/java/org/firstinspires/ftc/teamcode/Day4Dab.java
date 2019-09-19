package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Day 4 Dab")
//@Disabled
public class Day4Dab extends Day3AutoMethodLibrary {

    @Override
    public void runOpMode() {

        hardwareMap();

        waitForStart();
        //Put your autonomous code after this line

        drive(62,0.5);
        waitSec(1);
        absoluteTurn(-90);
        waitSec(1);
        drive(48,0.5);
        waitSec(1);
        absoluteTurn(180);
        waitSec(1);
        drive(37,0.5);


    }
}
