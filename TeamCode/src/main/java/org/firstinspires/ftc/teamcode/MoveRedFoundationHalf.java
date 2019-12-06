package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "MoveRedFoundationHalf")
//@Disabled0
public class MoveRedFoundationHalf extends MethodLibrary {

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
        drive(23,driveSpeed);
        absoluteTurn(-90);
        drive(28,driveSpeed);
        absoluteTurn(0);
        drive(26-robotLength,driveSpeed);
        //attach to foundation
        disengageHook();
        drive(1,driveSpeed/2);
        engageHook();
        //drive to building zone
        reverse(58-robotLength,dragSpeed);
        //detach foundation
        disengageHook();
        reverse(.5,driveSpeed);
        engageHook();
    }

}

