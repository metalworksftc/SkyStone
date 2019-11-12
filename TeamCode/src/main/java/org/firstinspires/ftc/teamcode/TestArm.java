package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "TestArm")
//@Disabled
public class TestArm extends MethodLibrary {

    @Override
    public void runOpMode() {

        hardwareMap();

        waitForStart();
        //Put your autonomous code after this line
//        raise(6);
//        waitSec(3);
        extend(5);
//        waitSec(3);
//        grab();
        waitSec(2);
//        release();
//        waitSec(3);
          retract(2);
//        waitSec(3);
//        lower(6);





    }

}
