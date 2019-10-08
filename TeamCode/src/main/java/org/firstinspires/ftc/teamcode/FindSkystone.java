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

        drive(43-robotLength,driveSpeed);
        absoluteTurn(-90);

        waitSec(1);

        while (cs.alpha() > 34 && cs.red() >   14) {
            reverse(1, 0.35);
            telemetry.addLine("Waiting for Skystone");
            telemetry.update();

        }
            while (true) {
                String message = cs.alpha() + " " + cs.green() + " " + cs.blue() + " " + cs.red();
                telemetry.addLine(message);
                telemetry.update();
                cs.alpha();
            }



    }

}

