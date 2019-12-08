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

        double driveSpeed=0.8;
        double robotLength=17.5;
        double dragSpeed=0.8;

        //drive to foundation
        disengageHook();
        drive(20,driveSpeed);
        absoluteTurnPower(90,0.3);
        drive(40,driveSpeed);
        absoluteTurnPower(0,0.3);
        drive(40-robotLength,driveSpeed);
        //attach to foundation
        disengageHook();
        drive(3,driveSpeed/2);
        engageHook();
        //drive to building zone
        reverse(66-robotLength,dragSpeed);
        //detach foundation
        disengageHook();
        //parking
        absoluteTurn(90);
        reverse(29,driveSpeed);
        absoluteTurn(0);
        drive(20,0.5);
        absoluteTurn(90);
        reverse(10,driveSpeed);

    }

}

