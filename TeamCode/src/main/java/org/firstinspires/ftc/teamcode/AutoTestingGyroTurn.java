/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "AutoTestingGyroTurn", group = "MecanumRobot")
@Disabled
public class AutoTestingGyroTurn extends AutoMethodLib {

    public void runOpMode() {
        //L: -57
        //C: -90
        //R: -125.625
        hardwareMW();
        //tfod.activate();
        while(!isStarted()) {
            telemetry.addLine("Initialized");
            telemetry.update();
        }
        waitSec(2);
        gyroTurnPID(90, "left");
        waitSec(2);
        gyroTurnPID(45, "right");
        /*waitSec(2);
        detectMinerals2CraterConfidence();
        telemetry.addLine("Objects detected" + tfod.getRecognitions().size());
        int i=0;
        for(Recognition recognition : tfod.getRecognitions()) {
            i++;
            telemetry.addLine("Object "+i+" Bottom: "+recognition.getBottom() + " | Width: "+recognition.getImageWidth());
            telemetry.addLine("Object "+i+" Label: "+recognition.getLabel());
        }
        telemetry.update();
        while (!gamepad1.a) {
        }
        while(!gamepad1.a) {
            telemetry.addLine("Objects detected" + tfod.getUpdatedRecognitions().size());
            int j=0;
            for(Recognition recognition : tfod.getUpdatedRecognitions()) {
                j++;
                telemetry.addLine("Object "+j+" Bottom: "+recognition.getBottom() + " | Width: "+recognition.getImageWidth());
                telemetry.addLine("Object "+j+" Label: "+recognition.getLabel());
            }
            telemetry.update();
        }*/
        /*while (opModeIsActive()) {
            int PID = 0;
           // while (!gamepad1.a && !gamepad1.left_bumper && !gamepad1.right_bumper && opModeIsActive() && gamepad1.left_trigger != 0
                 //   && gamepad1.right_trigger != 0) {
                if(gamepad1.x)
                    PID = 0;
                else if(gamepad1.y)
                    PID = 1;
                else if(gamepad1.b)
                    PID = 2;
                P += is(PID == 0) * (0.0000001 * is(gamepad1.dpad_up) - 0.0000001 * is(gamepad1.dpad_down));
                I += is(PID == 1) * (0.0000001 * is(gamepad1.dpad_up) - 0.0000001 * is(gamepad1.dpad_down));
                D += is(PID == 2) * (0.0000005 * is(gamepad1.dpad_up) - 0.0000005 * is(gamepad1.dpad_down));

                if(gamepad1.left_stick_y != 0) {
                    if(PID == 0) P =  (float) 0.0135;
                    else if(PID == 1) I = (float) 0.0005;
                    else D = (float) 0.00545;
                }
                telemetry.addLine("P: " + P);
                telemetry.addLine("I: " + I);
                telemetry.addLine("D: " + D);
                telemetry.addLine("Heading: "+getHeading());
                telemetry.update();

            if(gamepad1.left_trigger != 0) gyroTurnPID(45, "left");
            if(gamepad1.right_trigger != 0) gyroTurnPID(45, "right");
            if(gamepad1.left_bumper) gyroTurnPID(90, "left");
            if(gamepad1.right_bumper) gyroTurnPID(90, "right");
            if(gamepad1.a) encoderStrafeLeft(24, 0.6);

        }*/
    }
}
