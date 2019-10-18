package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "FindSkystone2")
//@Disabled
public class FindSkystone2 extends MethodLibrary {


    @Override
    public void runOpMode() {

        hardwareMap();

        waitForStart();
        //Put your autonomous code after this line
        drive(30.5, 0.5);
        servo.setPosition(0);
        waitSec(3);
        int block1 = cs.alpha();
        servo.setPosition(1);
        waitSec(3);
        int block3 = cs.alpha();
        int skystone = -1;
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


        while (true) {
            telemetry.addLine("skystone" + " is " + (skystone));
            telemetry.update();
            cs.alpha();
        }

    }

    private void goToStone3() {
        reverse(3,0.5);
        absoluteTurn(90);
        drive(3,0.5);
        absoluteTurn(180);
        reverse(3,0.5);
    }

    private void goToStone2() {
        reverse(3,0.5);
        absoluteTurn(-90);
        drive(1.5,0.5);
        absoluteTurn(180);
        reverse(3,0.5);
    }

    private void goToStone1() {
        reverse(3,0.5);
        absoluteTurn(-90);
        drive(10,0.5);
        absoluteTurn(180);
        reverse(3,0.5);
    }

    private boolean isBlack(int alpha) {
        return alpha < 100;
    }

}