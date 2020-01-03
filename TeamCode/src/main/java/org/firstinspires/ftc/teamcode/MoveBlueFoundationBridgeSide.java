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
        drive(20,driveSpeed);
        absoluteTurnPower(90,0.3);
        drive(34,driveSpeed);
        absoluteTurnPower(0,0.3);
        drive(21,driveSpeed);
        //attach to foundation
//        disengageHook();
//        drive(3,driveSpeed/2);
        engageHook();
        //drive to building zone
        reverse(44,dragSpeed);
        //detach foundation
        disengageHook();
        //parking
        absoluteTurnPower(0,0.3);
        strafeLeft(27,0.5);
        drive(31,driveSpeed);
        strafeLeft(17,0.5);

//      Move Blue Foundation Bridge Side
    }

}

