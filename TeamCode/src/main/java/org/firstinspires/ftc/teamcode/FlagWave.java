package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "FW")
//@Disabled
public class FlagWave extends MethodLibrary {

    @Override
    public void runOpMode() {

        hardwareMap();
        tailServo.setPosition(0);

        waitForStart();
        //Put your autonomous code after this line

        int count = 0;

        //tailServo.scaleRange(0, 0);
        while (count < 60) {
            tailServo.setPosition(0);
            waitSec(1);
            tailServo.setPosition(1);
            waitSec(1);
            count++;

        }


    }
}
