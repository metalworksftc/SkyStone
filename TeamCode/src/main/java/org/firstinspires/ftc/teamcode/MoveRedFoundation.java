package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "MoveRedFoundation")
//@Disabled
public class MoveRedFoundation extends MethodLibrary {

    @Override
    public void runOpMode() {

        hardwareMap();

        waitForStart();
        //Put your autonomous code after this line


        double driveSpeed=0.5;
        double robotLength=17.5;
        double dragSpeed=0.8;

        //drive to foundation
        engageHook();
        drive(24,driveSpeed);
        absoluteTurn(-90);
        drive(31,driveSpeed);
        absoluteTurn(0);
        drive(26-robotLength,driveSpeed);
        //attach to foundation
        disengageHook();
        drive(3.5,driveSpeed);
        engageHook();
        //drive to building zone
        reverse(58-robotLength,dragSpeed);
        //detach foundation
        disengageHook();
        reverse(.5,driveSpeed);
        engageHook();
        //parking
        absoluteTurn(-90);
        reverse(53,driveSpeed);


    }

}

