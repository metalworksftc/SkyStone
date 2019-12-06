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
        leftTailServo.setPosition(0.6);
        rightTailServo.setPosition(0.6);
        drive(20,driveSpeed);
        absoluteTurn(90);
        drive(25,driveSpeed);
        absoluteTurn(0);
        drive(24-robotLength,driveSpeed);
        //attach to foundation
        disengageHook();
        drive(3,driveSpeed/2);
        engageHook();
        //drive to building zone
        reverse(57-robotLength,dragSpeed);
        //detach foundation
        disengageHook();
        reverse(.5,driveSpeed);
        engageHook();
        //parking
        absoluteTurn(90);
        reverse(29,driveSpeed);
        absoluteTurn(0);
        drive(20,0.5);
        absoluteTurn(90);
        reverse(10,driveSpeed);

    }

}

