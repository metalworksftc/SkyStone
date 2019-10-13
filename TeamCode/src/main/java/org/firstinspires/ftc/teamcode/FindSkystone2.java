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
        servo.setPosition(0.5);
        drive(30.5, 0.5);
        servo.setPosition(0);
        waitSec(1);
        int block1 = cs.alpha();
        servo.setPosition(1);
        waitSec(1);
        int block3 = cs.alpha();

        while (true) {
            telemetry.addLine("Block 1 " + block1 + " Block 3 " + block3);
            telemetry.update();
            cs.alpha();
        }

    }

    private boolean isBlack() {
        return cs.alpha() > 32 && cs.red() >   14;
    }

}