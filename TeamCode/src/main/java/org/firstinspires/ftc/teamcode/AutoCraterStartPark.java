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

@Autonomous(name = "Crater Start: Park", group = "MecanumRobot")
//@Disabled
public class AutoCraterStartPark extends AutoMethodLib {

    public void runOpMode() {
        hardwareMW();
        telemetry.update();
        tfod.activate();
        while(!opModeIsActive() && !isStopRequested() && !isStarted()) {
            hardBrake(liftM);
            markerServo.setPosition(0.6);
            telemetry.addLine("I stopped a crash! Yaaaay");
            System.out.println(1+(1+1)+(1+1+1)+(1+1+1+1)+(1+1+1+1+1));
            System.out.println();
            detectMinerals2CraterConfidence();
            telemetry.update();
        }
        detectMinerals2CraterConfidence();
        liftM.setPower(1);
        waitSec(1.2);
        liftM.setPower(0);
        waitSec(0.5);
        newEncoderTank(-4, -0.8);
        waitSec(0.2);
        liftM.setPower(-1);
        encoderStrafeRight(24, 0.7);
        waitSec(0.2);
        newEncoderTank(4, 0.6);
        liftM.setPower(0);
        if(loc < 0) {
            absoluteGyroTurn(-55);
            waitSec(0.2);
            newEncoderTank(26, 0.7);
            waitSec(0.2);
            absoluteGyroTurn(-45);
            waitSec(0.2);
            newEncoderTank(-18, -0.7);
        } else if(loc == 0) {
            absoluteGyroTurn(-85);
            waitSec(0.2);
            newEncoderTank(23, 0.7);
            waitSec(0.2);
            absoluteGyroTurn(-90);
            waitSec(0.2);
            newEncoderTank(-13, -0.7);
        } else {
            absoluteGyroTurn(-117);
            waitSec(0.2);
            newEncoderTank(26, 0.7);
            waitSec(0.2);
            absoluteGyroTurn(-135);
            waitSec(0.2);
            newEncoderTank(-18, -0.7);
        }
        absoluteGyroTurn(170 - 160);
        waitSec(0.2);
        newEncoderTank(-36, -0.7);
        waitSec(0.2);
        absoluteGyroTurn(-135 + 90);
        waitSec(0.2);
        timeStrafeLeft(1.5, 0.5);
        waitSec(0.2);
        newEncoderTank(13, 0.5);
        waitSec(1);
        markerServo.setPosition(0);
        waitSec(1.5);
        markerServo.setPosition(0.6);
    }
}
