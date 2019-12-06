package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "CalibrateStrafe")
//@Disabled
public class CalibrateStrafe extends MethodLibrary {

    @Override
    public void runOpMode() {

        hardwareMap();

        waitForStart();
        strafeLeft(24, 0.8);
        waitSec(2.5);
        strafeRight(24,0.8);
    }

}

