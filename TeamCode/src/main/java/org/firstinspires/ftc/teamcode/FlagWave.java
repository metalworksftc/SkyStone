package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "FW")
//@Disabled
public class FlagWave extends MethodLibrary {

    @Override
    public void runOpMode() {

        hardwareMap();
        servo.setPosition(0);

        waitForStart();
        //Put your autonomous code after this line

        int count = 0;

        //servo.scaleRange(0, 0);
        while (count < 60) {
            servo.setPosition(0);
            waitSec(1);
            servo.setPosition(1);
            waitSec(1);
            count++;

        }


    }
}
