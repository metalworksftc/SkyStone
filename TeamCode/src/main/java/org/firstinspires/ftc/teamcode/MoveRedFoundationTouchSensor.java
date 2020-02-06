package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "MoveRedFoundationTouchSensor")
//@Disabled
public class MoveRedFoundationTouchSensor extends MethodLibrary {

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
        drive(21,driveSpeed);
        strafeLeft(25,driveSpeed);
        driveBump(40,driveSpeed);
       //attach to foundation
        engageHook();
        //drive to building zone
        reverse(57,dragSpeed);
        //detach foundation
        disengageHook();
        //parking
        absoluteTurnPower(-20,0.7);
        absoluteTurnPower(0,0.3);
        strafeRight(24,0.6);
        drive(1,driveSpeed);
        strafeRight(20,0.6);
//      Move Red Foundation
    }

}
