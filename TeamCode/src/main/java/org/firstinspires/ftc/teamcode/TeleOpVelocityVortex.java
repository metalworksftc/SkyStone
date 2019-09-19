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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//Created using http://thinktank.wpi.edu/resources/346/ControllingMecanumDrive.pdf and math.

@TeleOp(name = "TeleOpVV", group = "Iterative Opmode")
//@Disabled
public class TeleOpVelocityVortex extends OpMode {

    DcMotor leftMotor, rightMotor;
    DcMotor flywheelMotor, beaterMotor;
    Servo accessServo;
    ElapsedTime runtime = new ElapsedTime();
    double flywheelPower = 0;
    double beaterPower = 0;

    int beaterCount, flyCount, servoCount;

    boolean servoUp = false;
    double SERVO_CLOSED = 0.48;
    double SERVO_OPENED = 0.12;

    @Override
    public void init() {
        leftMotor = hardwareMap.dcMotor.get("lm");
        rightMotor = hardwareMap.dcMotor.get("rm");
        flywheelMotor = hardwareMap.dcMotor.get("fm");
        beaterMotor = hardwareMap.dcMotor.get("bm");
        accessServo = hardwareMap.servo.get("s");

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        accessServo.setPosition(SERVO_CLOSED);
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

        leftMotor.setPower(gamepad1.left_stick_y);
        rightMotor.setPower(gamepad1.right_stick_y);

        if(flyCount < 60) flyCount++;
        else if(gamepad1.a || gamepad1.b || gamepad1.x) {
            flyCount = 0;
            if (gamepad1.a) flywheelPower = 1;
            else if (gamepad1.b) flywheelPower = 0.75;
            else flywheelPower = 0;
        }
        flywheelMotor.setPower(flywheelPower);

        if(beaterCount < 60) beaterCount++;
        else if(gamepad1.dpad_up || gamepad1.dpad_down) {
            if (gamepad1.dpad_up && beaterPower != -1) beaterPower = -1;
            else if (gamepad1.dpad_down && beaterPower != 1) beaterPower = 1;
            else beaterPower = 0;
        }
        beaterMotor.setPower(beaterPower);

        if(gamepad1.right_trigger != 0) {
            servoUp = true;
            accessServo.setPosition(SERVO_OPENED);
        }
        if(servoCount < 200 && servoUp) servoCount++;
        else if(servoCount == 200 && servoUp) {
            servoCount = 0;
            servoUp = false;
            accessServo.setPosition(SERVO_CLOSED);
        }

        telemetry.addLine("Servo Count: "+servoCount);
        telemetry.update();


    }

    @Override
    public void stop() {


    }
}
