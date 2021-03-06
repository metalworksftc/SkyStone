package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "MoveBlueFoundation")
@Disabled
public class MoveBlueFoundation extends MethodLibrary {

    @Override
    public void runOpMode() {

        hardwareMap();

        waitForStart();
        //Put your autonomous code after this line

        //drive to foundation
        disengageHook();
        drive(22,driveSpeed);
        strafeRight(25, driveSpeed);
        drive(22 ,driveSpeed);
        //attach to foundation
        engageHook();
        //drive to building zone
        reverse(57,dragSpeed);
        //detach foundation
        disengageHook();
        //parking
        absoluteTurnPower(-20,0.7);
        absoluteTurnPower(0,0.3);
        strafeLeft(30,0.6);
        drive(1,driveSpeed);
        strafeLeft(17,0.6);
//      Move Blue Foundation
    }

}