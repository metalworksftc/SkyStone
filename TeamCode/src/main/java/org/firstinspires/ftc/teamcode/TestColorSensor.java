package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "TestColorSensor")
//@Disabled
public class TestColorSensor extends MethodLibrary {

    @Override
    public void runOpMode() {

        hardwareMap();

        waitForStart();
        //Put your autonomous code after this line


        while (true) {
            String message = cs.alpha() + " " + cs.green() + " " + cs.blue() + " " + cs.red();
//            String message = String.valueOf(testsensor.getLightDetected());
            telemetry.addLine(message);
            telemetry.update();
//            cs.alpha();
            testsensor.enableLed(true);
            testsensor.getLightDetected();

        }



    }

}

