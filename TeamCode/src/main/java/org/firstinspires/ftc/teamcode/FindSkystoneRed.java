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
        tailServo.setPosition(1);
        waitSec(3);
        int block1 = cs.alpha();
        tailServo.setPosition(0);
        waitSec(3);
        int block3 = cs.alpha();
        int skystone;
        if (isBlack(block1)) {
            skystone = 1;
            goToStone1();
        }
        else if (isBlack(block3)) {
            skystone = 3;
            goToStone3();
        }
        else {
            skystone = 2;
            goToStone2();
        }
        telemetry.addLine(String.valueOf(skystone));
        telemetry.update();
    }

    private void goToStone3() {
        reverse(4,0.5);
        absoluteTurn(90);
        drive(3,0.5);
        absoluteTurn(180);
        pickUpBlock();
        deliverBlock(70);
    }

    private void goToStone2() {
        reverse(4,0.5);
        absoluteTurn(180);
        pickUpBlock();
        deliverBlock(65);
    }

    private void goToStone1() {
        reverse(4,0.5);
        absoluteTurn(-90);
        drive(7,0.5);
        absoluteTurn(180);
        pickUpBlock();
        deliverBlock(45);
    }

    private void pickUpBlock() {
        raise(4.5);
        extend(53);
        lower(4.5);
        grab();
    }

    private void deliverBlock(double dist) {
        drive(5,0.5);
        absoluteTurn(90);
        reverse(dist,0.8);
        //parks on line
        drive(20,0.7);
    }

    private boolean isBlack(int alpha) {
        return alpha < 100;
    }

}