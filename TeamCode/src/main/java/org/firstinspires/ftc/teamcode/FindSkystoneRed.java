package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "FindSkystoneRed")
//@Disabled
public class FindSkystoneRed extends MethodLibrary {

    @Override
    public void runOpMode() {

        hardwareMap();

        waitForStart();
        //Put your autonomous code after this line
        drive(27.5, 0.6);
        waitSec(3);
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
        strafeRight(12,0.5);
        pickUpBlock();
        deliverBlock(70,"r");
    }

    private void goToStone2() {
        strafeRight(3,0.5);
        pickUpBlock();
        deliverBlock(65,"r");
    }

    private void goToStone1() {
        strafeLeft(3,0.5);
        absoluteTurn(180);
        pickUpBlock();
        deliverBlock(45,"r" );
    }

    public void pickUpBlock() {
        raise(4.5);
        extend(7.5);
        lower(4.5);
        grab();
    }



    private boolean isBlack(int alpha) {
        return alpha < 100;
    }

}