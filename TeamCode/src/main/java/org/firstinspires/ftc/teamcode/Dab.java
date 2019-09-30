package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Dab")
//@Disabled
public class Dab extends MethodLibrary {

    @Override
    public void runOpMode() {

        hardwareMap();

        waitForStart();
        //Put your autonomous code after this line

        drive(16,0.5);
        absoluteTurn(-90);
        drive(13,0.5);
        absoluteTurn(180);
        drive(10,0.5);


    }
}
