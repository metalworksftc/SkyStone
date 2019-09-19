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

@Autonomous(name = "AutoTestingGyroStraight", group = "MecanumRobot")
@Disabled
public class AutoTestingGyroStraight extends AutoMethodLib {

    public void runOpMode() {
        hardwareMW();
        while(!isStarted()) {
            telemetry.addLine("Initialized");
            telemetry.update();
        }

        while (opModeIsActive()) {
            int PID = 0;
            if(gamepad1.x)
                PID = 0;
            else if(gamepad1.y)
                PID = 1;
            else if(gamepad1.b)
                PID = 2;
            kP += is(PID == 0) * (0.00001 * is(gamepad1.dpad_up) - 0.00001 * is(gamepad1.dpad_down));
            kI += is(PID == 1) * (0.0000001 * is(gamepad1.dpad_up) - 0.0000001 * is(gamepad1.dpad_down));
            kD += is(PID == 2) * (0.000005 * is(gamepad1.dpad_up) - 0.000005 * is(gamepad1.dpad_down));

            if(gamepad1.left_stick_y != 0) {
                if(PID == 0) kP = 0.067;
                else if(PID == 1) kI = 0;
                else kD = 0;
            }

            telemetry.addLine("kP: " + kP);
            telemetry.addLine("kI: " + kI);
            telemetry.addLine("kD: " + kD);
            telemetry.addLine("Heading: "+getHeading());
            telemetry.update();

            if(gamepad1.left_bumper) gyroStraight(48, 0.6);
            if(gamepad1.right_bumper) gyroStraight(-48, -0.6);
            if(gamepad1.a) encoderStrafeLeft(24, 0.6);
        }
    }
}
