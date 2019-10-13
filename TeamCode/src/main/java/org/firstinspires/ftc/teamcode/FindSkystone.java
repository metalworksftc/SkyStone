package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.sql.Driver;

@Autonomous(name = "FindSkystone")
//@Disabled
public class FindSkystone extends MethodLibrary {


    @Override
    public void runOpMode() {

        hardwareMap();

        waitForStart();
        //Put your autonomous code after this line

        double driveSpeed=0.5;
        double robotLength=17.5;
        engageHook();

        drive(50-robotLength,driveSpeed);
        absoluteTurn(-90);
        reverse(20,0.5);
        disengageHook();

        waitSec(1);

        while (isBlack()) {
            reverse(3, 0.35);
            telemetry.addLine("Waiting for Skystone");
            telemetry.update();

        }
        drive(11,0.5);
        absoluteTurn(180);
        waitSec(3);


    }

    private boolean isBlack() {
        return cs.alpha() > 32 && cs.red() >   14;
    }

}

