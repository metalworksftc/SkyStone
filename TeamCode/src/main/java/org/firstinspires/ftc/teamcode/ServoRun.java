package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "ServoRun")
//@Disabled
public class ServoRun extends MethodLibrary {

    @Override
    public void runOpMode() {

        hardwareMap();

        waitForStart();
        //Put your autonomous code after this line

        drive(16,0.5);
        servo.setPosition(1);
        servo.setPosition(0);
        reverse(16,0.5);

        stop();
    }
}

