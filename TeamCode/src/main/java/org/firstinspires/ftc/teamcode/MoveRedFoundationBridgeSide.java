package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "MoveRedFoundationBridgeSide")
//@Disabled
public class MoveRedFoundationBridgeSide extends MethodLibrary {

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
        absoluteTurnPower(-90,0.3);
        drive(45,driveSpeed);
        absoluteTurnPower(0,0.3);
        drive(44-robotLength,driveSpeed);
        //attach to foundation
//        disengageHook();
//        drive(3,driveSpeed/2);
        engageHook();
        //drive to building zone
        reverse(68-robotLength,dragSpeed);
        //detach foundation
        disengageHook();
        //parking
        absoluteTurnPower(0,0.3);
        strafeRight(27,0.5);
        drive(36.5,driveSpeed);
        strafeRight(14,0.5);

    }

}

