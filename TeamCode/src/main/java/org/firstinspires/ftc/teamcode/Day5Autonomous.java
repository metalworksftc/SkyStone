package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Day 5 Autonomous")
//@Disabled
public class Day5Autonomous extends Day5AutoMethodLibrary {

    @Override
    public void runOpMode() {

        hardwareMap();

        waitForStart();
        //Put your autonomous code after this line
//        drive(62, 0.5);
//        waitSec(1);
//        absoluteTurn(-135);
//        waitSec(1);
//        drive(67, 0.5);
//        waitSec(1);
//        absoluteTurn(0);
//        waitSec(1);
//        drive(46,0.5);
//        waitSec(1);
//        reverse(46,0.5);
//        waitSec(1);
//        absoluteTurn(-135);
//        waitSec(1);
//        reverse(67,0.5);
//        waitSec(1);
//        absoluteTurn(0);
//        waitSec(1);
//        reverse(62,0.5);

        telemetry.addLine("about to set position");
        telemetry.update();

        int count = 0;

        //tailServo.scaleRange(0, 0);
        while (count < 1) {
            servo.setPosition(0);
            waitSec(2);
            servo.setPosition(0);
            waitSec(2);
            count++;
        }





//        double position = tailServo.getPosition();
//        telemetry.addLine(String.valueOf(position));
//        telemetry.update();
//        waitSec(2);
//        telemetry.addLine("done setting position");
//        telemetry.update();
//        waitSec(2);



    }
}
