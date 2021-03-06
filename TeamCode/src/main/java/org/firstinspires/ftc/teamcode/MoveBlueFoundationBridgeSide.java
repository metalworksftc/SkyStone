package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "MoveBlueFoundationBridgeSide")
@Disabled
public class MoveBlueFoundationBridgeSide extends MethodLibrary {

    @Override
    public void runOpMode() {

        hardwareMap();

        waitForStart();
        //Put your autonomous code after this line

        //drive to foundation
        disengageHook();
        drive(22,driveSpeed);
        strafeRight(25, driveSpeed);
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
        strafeLeft(24,0.5);
        drive(38,driveSpeed);
        strafeLeft(20,0.5);
//      Move Blue Foundation Bridge Side
    }

}