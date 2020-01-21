package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "TestTurn")
//@Disabled
public class TestTurn extends MethodLibrary {

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
        absoluteTurnPower(-90,0.3);
        drive(24,driveSpeed);
        absoluteTurnPower(0,0.3);
        drive(14,driveSpeed);
        //attach to foundation
        engageHook();
        //drive to building zone
        reverse(20,dragSpeed);
        absoluteTurnPower(-90,dragSpeed);
        //detach foundation
        disengageHook();
        //parking
        reverse(44,dragSpeed);
//        absoluteTurnPower(-30,0.7);
//        absoluteTurnPower(0,0.3);
//        strafeRight(46,0.6);

//      Move Red Foundation
    }

}