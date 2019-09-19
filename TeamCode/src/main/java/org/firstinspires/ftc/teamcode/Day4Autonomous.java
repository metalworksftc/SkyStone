package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Day 4 Autonomous")
//@Disabled
public class Day4Autonomous extends Day3AutoMethodLibrary {

    @Override
    public void runOpMode() {

        hardwareMap();

        waitForStart();
        //Put your autonomous code after this line
        drive(62, 0.5);
        waitSec(1);
        absoluteTurn(-135);
        waitSec(1);
        drive(67, 0.5);
        waitSec(1);
        absoluteTurn(0);
        waitSec(1);
        drive(46,0.5);
        waitSec(1);
        reverse(46,0.5);
        waitSec(1);
        absoluteTurn(-135);
        waitSec(1);
        reverse(67,0.5);
        waitSec(1);
        absoluteTurn(0);
        waitSec(1);
        reverse(62,0.5);



    }
}
