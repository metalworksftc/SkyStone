package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Day 3 Autonomous")
//@Disabled
public class Day3Autonomous extends Day3AutoMethodLibrary {

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
        absoluteTurn(90);
        waitSec(1);
        drive(101,0.5);
        waitSec(1);
        absoluteTurn(0 );

    }
}
