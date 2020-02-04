package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "TestTouchSensor")
//@Disabled
public class TestTouchSensor extends MethodLibraryTest {

    @Override
    public void runOpMode() {

        hardwareMap();

        waitForStart();
        while (true) {
            if (touchSensor.isPressed())
                telemetry.addLine("Sensor is pushed");
            else
                telemetry.addLine("Sensor is not pushed");
            telemetry.update();
        }
    }

}
