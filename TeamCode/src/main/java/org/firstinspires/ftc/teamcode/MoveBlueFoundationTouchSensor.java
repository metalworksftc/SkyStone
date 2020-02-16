package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "MoveBlueFoundation")
//@Disabled
public class MoveBlueFoundationTouchSensor extends MethodLibrary {

    @Override
    public void runOpMode() {

        hardwareMap();

        waitForStart();
        //Put your autonomous code after this line

        //drive to foundation
        disengageHook();
        drive(intApproach + 2,driveSpeed);
        strafeRight(10,driveSpeed);
        double driveBumpDist = driveBump(30,driveSpeed);
       //attach to foundation
        engageHook();
        //drive to building zone
        reverse(driveBumpDist + 1 + intApproach,dragSpeed);
        //detach foundation
        disengageHook();
        //parking
        absoluteTurnPower(-20,0.7);
        absoluteTurnPower(0,0.3);
        strafeLeft(24,0.6);
        drive(4,driveSpeed);
        strafeLeft(16,0.6);
//      Move Blue Foundation
    }

}