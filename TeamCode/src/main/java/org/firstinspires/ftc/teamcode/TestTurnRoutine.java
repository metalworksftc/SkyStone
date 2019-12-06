package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "MoveRedFoundation")
//@Disabled
public class TestTurnRoutine extends MethodLibrary {

    @Override
    public void runOpMode() {

        hardwareMap();

        waitForStart();
        //Put your autonomous code after this line

        double driveSpeedFast=0.8;
        double driveSpeedSlow=0.3;
        double robotLength=17.5;
        double dragSpeed=0.8;

        //drive to foundation

        leftTailServo.setPosition(0.6);
        rightTailServo.setPosition(0.6);

        drive(20,driveSpeedFast);
        absoluteTurn(-75);
        drive(25,driveSpeedSlow);
        absoluteTurn(-15);
        drive(37-robotLength,driveSpeedFast);
        //attach to foundation
        disengageHook();
        drive(3,driveSpeedSlow/2);
        engageHook();
        //drive to building zone
        reverse(57-robotLength,dragSpeed);
        //detach foundation
        disengageHook();
        reverse(.5,driveSpeedFast);
        engageHook();
        //parking
        strafeRight(53,0.8);


    }

}

