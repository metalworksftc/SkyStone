package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "TouchSensorCalibration")
//@Disabled
public class TouchSensorCalibration extends MethodLibrary {

    @Override
    public void runOpMode() {

        hardwareMap();

        waitForStart();
        driveBump(1000,driveSpeed);
    }

}
