package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous//(name = "TestTurnRoutine")
//@Disabled
public class TestTurnRoutine extends MethodLibrary {

    @Override
    public void runOpMode() {

        hardwareMap();

        waitForStart();
        //Put your autonomous code after this line

        double driveSpeedFast=0.8;
        double driveSpeedSlow=0.3;
        double robotLength=17.5;
        double dragSpeed=0.8;

        //drive to foundation

       absoluteTurnPower(90,0.4);
       waitSec(5);
       absoluteTurnPower(0,0.4);
    }

}

