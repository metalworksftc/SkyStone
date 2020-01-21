package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "MoveBlueFoundation")
//@Disabled
public class MoveBlueFoundation extends MethodLibrary {

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
        strafeRight(25, driveSpeed);
        drive(16 ,driveSpeed);
        //attach to foundation
        engageHook();
        //drive to building zone
        reverse(48,dragSpeed);
        //detach foundation
        disengageHook();
        //parking
        absoluteTurnPower(15,0.7);
        absoluteTurnPower(0,0.7);
        drive(2 ,driveSpeed);
        strafeLeft(45,0.6);

//      Move Blue Foundation
    }

}