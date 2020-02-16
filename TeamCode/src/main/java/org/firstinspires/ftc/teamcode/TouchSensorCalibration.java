package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "TouchSensorCalibration")
//@Disabled
public class TouchSensorCalibration extends MethodLibrary {

    @Override
    public void runOpMode() {

        hardwareMap();

        waitForStart();
        double inTraveled = driveBump(1000,driveSpeed);
        double disTraveled = inTraveled * CALIBRATION_COUNTS/DRIVE_CALIBRATION;
        telemetry.addLine("Distance in counts " + disTraveled);
        telemetry.addLine("Distance in inches " + inTraveled);
        telemetry.update();
        waitSec(5);
    }

}
