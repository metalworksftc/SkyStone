package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "MoveRedFoundationBridgeSide")
@Disabled
public class MoveRedFoundationBridgeSide extends MethodLibrary {

    @Override
    public void runOpMode() {

        hardwareMap();

        waitForStart();
        //Put your autonomous code after this line

        //drive to foundation
        disengageHook();
        drive(22,driveSpeed);
        strafeLeft(25, driveSpeed);
        drive(22,driveSpeed);
        //attach to foundation
        engageHook();
        //drive to building zone
        reverse(48,dragSpeed);
        //detach foundation
        disengageHook();
        //parking
        absoluteTurnPower(-20,0.7);
        absoluteTurnPower(0,0.3);
        strafeRight(24,0.6);
        drive(38,driveSpeed);
        strafeRight(20,0.5);

//      Move Red Foundation Bridge Side
    }

}