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

@TeleOp(name = "TeleOpCrater", group = "Iterative Opmode")
//@Disabled
public class TeleOpCrater extends TeleOpMethodLibrary {


    double intakeServoPower = 0;
    int dpadCooldown = 0;
    final int offset = 90;

    @Override
    public void init() {
        hardwareMWTeleOp();
        try {
            brakeMotors();
        } catch (Exception e) {
            telemetry.addLine(e.getCause()+" caused an error while braking the motors.");
        }

    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
        timeoutCheck.reset();
    }

    @Override
    public void loop() {

            liftM.setPower(are(gamepad1.left_bumper, gamepad1.right_bumper));
            telemetry.addLine("Lift Power" + liftM.getPower());
            /*if(dpadCooldown >= 30 && (gamepad2.dpad_up)) {
                dpadCooldown = 0;
                intakeServoPower = is(intakeServoPower == 0);
            } else dpadCooldown++;*/
            //intakeServo.setPower(-0.3905*is(intakeServoPower == 0) + is(intakeServoPower == 1));
        /*} catch (Exception e) {
            telemetry.addLine(e.getCause()+" caused an error while moving the lift motor.");
        }*/

        //tankDrive();
        //testSpeed();
        toggleDrive(offset);
        slowMode();
        hangDrive();
        if(!isControlling) controlArm();
        //scoreMinerals();

        /*try {
            if(gamepad1.a) markerServo.setPosition(1);
            if(gamepad1.x) markerServo.setPosition(0.2);
            //if(gamepad1.left_bumper) mineralServo.setPosition(MINERAL_SERVO_DOWN);
            //if(gamepad1.right_bumper) mineralServo.setPosition(MINERAL_SERVO_UP);
        } catch (Exception e) {
            telemetry.addLine(e.getCause()+" caused an error while moving the marker servo.");
        }*/

        try {

        } catch (Exception e) {

        }

        telemetry.addLine("LF: "+round(lFPow, 2)+" | LB: "+round(lBPow, 2));
        telemetry.addLine("RF: "+round(rFPow, 2)+" | RB: "+round(rBPow, 2));
        telemetry.addLine("Left Y: "+(gamepad1.left_stick_y)+" | Left X: "+(-gamepad1.left_stick_x));
        telemetry.addLine("isStrafing: "+strafing+" | slowMode: "+slowMode+" | Hang Drive: "+hangDrive);
        telemetry.addLine("Runtime: "+round(runtime.seconds(), 3) + " | " + step);
        telemetry.addLine("Limit Switch: " + armLimit.getState());
        //telemetry.addLine("Hue: "+getHue()+" | Saturation: "+hsvValues[1]+" | Value: "+hsvValues[2]);
        //telemetry.addLine("rtTriggerCD: "+dpadCooldown+" | SP: "+intakeServo.getPower());
        //telemetry.addLine("LM Position: "+liftM.getCurrentPosition());
    }

    @Override
    public void stop() {
        try {
            coastMotors();
        } catch (Exception e) {
            telemetry.addLine(e.getCause()+" caused an error while coasting the motors.");
        }

    }
}
