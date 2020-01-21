package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "MoveRedFoundation")
//@Disabled
public class MoveRedFoundation extends MethodLibrary {

    @Override
    public void runOpMode() {

        hardwareMap();

        waitForStart();
        //Put your autonomous code after this line

        double driveSpeed=0.5;
        double robotLength=17.5;
        double dragSpeed=0.8;

        //drive to foundation
        disengageHook();
        drive(20,driveSpeed);
        strafeLeft(25,driveSpeed);
        drive(9,driveSpeed);
        //attach to foundation
        engageHook();
        //drive to building zone
        reverse(48,dragSpeed);
        //detach foundation
        disengageHook();
        //parking
        absoluteTurnPower(-12,0.7);
        absoluteTurnPower(0,0.3);
        strafeRight(50,0.6);

//      Move Red Foundation
    }

}