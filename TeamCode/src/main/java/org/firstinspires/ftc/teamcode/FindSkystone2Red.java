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
        reverse(35, 0.6);
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
        drive(2,0.6);
        strafeLeft(8,0.5);
        pickUpBlock();
        deliverBlock(70,"r");
    }

    private void goToStone2() {
        drive(1.5,0.6);
        strafeLeft(3,0.5);
        pickUpBlock();
        deliverBlock(65,"r");
    }

    private void goToStone1() {
        drive(1,0.6);
        strafeRight(3,0.5);
        pickUpBlock();
        deliverBlock(55,"r");
    }

    public void pickUpBlock() {
        raise(5);
        extend(4);
        lower(5);
        grab();
    }
//    Find Skystone Red

    private boolean isBlack(int alpha) {
        return alpha < 100;
    }

}