package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "FindSkystone2Red")
//@Disabled
public class FindSkystone2Red extends MethodLibrary {

    @Override
    public void runOpMode() {

        hardwareMap();

        waitForStart();
        //Put your autonomous code after this line
        reverse(46, 0.6);
        reverse(3.5,0.4);
        waitSec(1);
        int block1 = rightColorSensor.alpha();
        int block2 = leftColorSensor.alpha();

        int skystone;
        if (isBlack(block1)) {
            skystone = 1;
            goToStone1();
        }
        else if (isBlack(block2)) {
            skystone = 2;
            goToStone2();
        }
        else {
            skystone = 3;
            goToStone3();
        }
        telemetry.addLine(String.valueOf(skystone));
        telemetry.update();
    }

    private void goToStone3() {
        waitSec(1);
        drive(.5,0.6);
        strafeLeft(8,0.5);
        reverse(1,0.6);
        pickUpBlock();
        deliverBlock(75,"r");
    }

    private void goToStone2() {
        drive(2,0.6);
        strafeRight(3,0.5);
        reverse(1,0.6);
        pickUpBlock();
        deliverBlock(65,"r");
    }

    private void goToStone1() {
        waitSec(1);
        drive(1,0.6);
        strafeRight(3,0.5);
        reverse(1,0.6);
        pickUpBlock();
        deliverBlock(55,"r");
    }

//    Find Skystone Red
}