package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "MoveRedFoundation")
//@Disabled
public class MoveRedFoundationTouchSensor extends MethodLibrary {

    @Override
    public void runOpMode() {

        hardwareMap();

        waitForStart();
        //Put your autonomous code after this line

        //drive to foundation
        disengageHook();
        drive(intApproach,driveSpeed);
        strafeLeft(12,driveSpeed);
        double driveBumpDist = driveBump(30,driveSpeed);
       //attach to foundation
        engageHook();
        //drive to building zone
        reverse(driveBumpDist + intApproach,dragSpeed);
        //detach foundation
        disengageHook();
        //parking
        absoluteTurnPower(-20,0.7);
        absoluteTurnPower(0,0.3);
        strafeRight(24,0.6);
        drive(4,driveSpeed);
        strafeRight(20,0.6);
//      Move Red Foundation
    }

}

