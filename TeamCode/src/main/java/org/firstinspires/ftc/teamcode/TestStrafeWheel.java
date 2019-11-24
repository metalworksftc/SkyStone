package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "TestStrafeWheel")
//@Disabled
public class TestStrafeWheel extends MethodLibrary {

    @Override
    public void runOpMode() {

        hardwareMap();

        waitForStart();
        drive(30,0.5);
        strafeLeft(10,0.5);
        strafeRight(10,0.5);
    }

}
