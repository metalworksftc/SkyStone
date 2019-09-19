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

@Autonomous(name = "TestAllSensors", group = "MecanumRobot")
@Disabled
public class TestAllSensors extends AutoMethodLib {

    int[] colors;
    float[] hsv;
    public void runOpMode() {
        waitForStart();
        while(opModeIsActive()) {
            /*colors = new int[] {cS.red(), cS.green(), cS.blue(), cS.alpha()};
            telemetry.addLine("R: "+colors[0]+" | G: "+colors[1]+" | B: "+colors[2]+" | A: "+colors[3]);
            telemetry.addLine("Sees: "+(cS.red() > cS.blue() ? (cS.red() > cS.green() ? "RED" : (cS.blue() > cS.green() ? "BLUE" : "GREEN")) : (cS.blue() > cS.green() ? "BLUE" : "GREEN")));
            telemetry.addLine("Distance (in.): " + round(dS.getDistance(DistanceUnit.INCH), 3));
            telemetry.addLine("Angle: "+getHeading());
*/
            telemetry.update();

        }
    }
}

//Read, move left for cube and right for ball
//Button to decide which to pick up before intake
//Correct: servo kick into bucket
//Wrong: keep running, restart
