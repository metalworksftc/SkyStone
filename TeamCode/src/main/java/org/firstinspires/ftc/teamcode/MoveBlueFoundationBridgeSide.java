package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "MoveBlueFoundationBridgeSide")
//@Disabled
public class MoveBlueFoundationBridgeSide extends MethodLibrary {

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
        absoluteTurnPower(12,0.7);
        absoluteTurnPower(0,0.3);
        strafeLeft(24,0.5);
        drive(38,driveSpeed);
        strafeLeft(20,0.5);

//      Move Blue Foundation Bridge Side
    }

}