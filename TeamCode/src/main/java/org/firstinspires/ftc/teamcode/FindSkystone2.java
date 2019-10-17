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
        }
        else if (isBlack(block3)) {
            skystone = 3;
        }
        else {
            skystone = 2;
        }


        while (true) {
            telemetry.addLine("skystone" + " is" + (skystone));
            telemetry.update();
            cs.alpha();
        }

    }

    private boolean isBlack(int alpha) {
        return alpha < 100;
    }

}