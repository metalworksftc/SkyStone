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

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//Created using http://thinktank.wpi.edu/resources/346/ControllingMecanumDrive.pdf and math.

@TeleOp(name = "TeleOpTesting", group = "Iterative Opmode")
//@Disabled
public class TeleOpTesting extends TeleOpMethodLibrary {

    boolean pivotIsPressed, slideIsPressed;
    double intakeServoPower = 0;
    int rtTriggerCD = 0;
    @Override
    public void init() {
        //hardwareMWTesting();
        hardwareMWTeleOp();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        tankDrive();
        slowMode();
        //toggleDrive();
        hangDrive();

        if(gamepad2.left_stick_y != 0) {
            pivotM.setPower(gamepad2.left_stick_y);
            pivotIsPressed = true;
        }
        if(gamepad2.left_stick_y == 0) {
            if(pivotIsPressed) {
                pivotIsPressed = false;
                pivotStartingEncoderCount = pivotM.getCurrentPosition();
            }
            hardBrakePivot(pivotM);
        }
        if(gamepad2.right_stick_y != 0) {
            storagePos += 0.001*isNeg(gamepad2.right_stick_y > 0);
        }
        storageServo.setPosition(storagePos);
        if(rtTriggerCD >= 30 && (gamepad2.right_trigger != 0)) {
            rtTriggerCD = 0;
            intakeServoPower = is(intakeServoPower == 0);
        } else rtTriggerCD++;
        //intakeServo.setPower(-0.3905*is(intakeServoPower == 0) + is(intakeServoPower == 1));

        if(gamepad2.dpad_up || gamepad2.dpad_down) {
            slideM.setPower(is(gamepad2.dpad_up) - is(gamepad2.dpad_down));
            slideIsPressed = true;
        }
        if(!gamepad2.dpad_up && !gamepad2.dpad_down) {
            if(slideIsPressed) {
                slideIsPressed = false;
                slideStartingEncoderCount = slideM.getCurrentPosition();
            }
            hardBrakePivot(slideM);
        }
        telemetry.addLine("SlowMode: "+slowMode+" | Hang Drive: "+hangDrive);
        telemetry.addLine("Storage Position: "+storageServo.getPosition());
        telemetry.addLine("Arm Position: "+pivotM.getCurrentPosition());
        telemetry.addLine("RtTriggerCt: "+rtTriggerCD);
        telemetry.addLine("SlideEnc: " + slideM.getCurrentPosition());
        //telemetry.addLine("Intake Pwr: "+intakeServo.getPower());
        telemetry.update();
    }

    @Override
    public void stop() {
    }
}
