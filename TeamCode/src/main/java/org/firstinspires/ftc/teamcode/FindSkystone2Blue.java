package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "FindSkystone2Blue")
//@Disabled
public class FindSkystone2Blue extends MethodLibrary {

    @Override
    public void runOpMode() {

        hardwareMap();

        waitForStart();
        //Put your autonomous code after this line
        reverse(44.5, 0.6);
        reverse(3.5,0.4);
        waitSec(1);
        int block2 = rightColorSensor.alpha();
        int block1 = leftColorSensor.alpha();

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
        strafeRight(8,0.5);
        pickUpBlock();
        deliverBlock(70,"l");
    }

    private void goToStone2() {
        drive(2,0.6);
        strafeLeft(3,0.5);
        pickUpBlock();
        deliverBlock(65,"l");
    }

    private void goToStone1() {
        waitSec(1);
        drive(1,0.6);
        strafeLeft(3,0.5);
        pickUpBlock();
        deliverBlock(55,"l");
    }

    public void pickUpBlock() {
        raise(5);
        extend(4);
        lower(5);
        grab();
    }
//    Find Skystone Blue
}