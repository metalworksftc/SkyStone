package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "MoveBlueFoundation")
//@Disabled
public class MoveBlueFoundation extends MethodLibrary {

    @Override
    public void runOpMode() {

        hardwareMap();

        waitForStart();
        //Put your autonomous code after this line


        double driveSpeed=0.5;
        double robotLength=17.5;
        double dragSpeed=0.8;

        //drive to foundation
        drive(24,driveSpeed);
        absoluteTurn(90);
        drive(29,driveSpeed);
        absoluteTurn(0);
        drive(28.5-robotLength,driveSpeed);
        //attach to foundation
        disengageHook();
        drive(.5,driveSpeed);
        engageHook();
        //drive to building zone
        reverse(57.5-robotLength,dragSpeed);
        //detach foundation
        disengageHook();
        reverse(.5,driveSpeed);
        engageHook();
        //parking
        absoluteTurn(90);
        reverse(52,driveSpeed);


    }

}

