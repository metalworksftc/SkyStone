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
        while (true){
            telemetry.addLine("left " + leftColorSensor.red() + " " + leftColorSensor.green() + " " + leftColorSensor.blue() + " " + leftColorSensor.alpha());
            waitSec(2);
            telemetry.addLine("right " + rightColorSensor.red() + " " + rightColorSensor.green() + " " + rightColorSensor.blue() + " " + rightColorSensor.alpha());
            telemetry.update();
        }
    }

}
