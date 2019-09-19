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

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

@TeleOp(name = "VVBarnstormers", group = "VVRobot")

public class VVTeleOp extends OpMode {

    ElapsedTime driveModeChanger = new ElapsedTime();
    int count = 0;
    double stickY = 0;
    double stickX = 0;
    DcMotor lm, rm, fm, bm;
    DcMotor[] drive = {lm, rm};
    Servo fs;

    ElapsedTime runtime = new ElapsedTime();

    /*BNO055IMU imu;
    Orientation angles = new Orientation();
    Acceleration gravity;
    boolean cheezyToggle = true;
    int hold = 0;
    float hsvValues[] = {0F, 0F, 0F};
    final float values[] = hsvValues;
    final double SCALE_FACTOR = 255;
    int relativeLayoutId;*/

    @Override
    public void init() {
        hardwareMWTeleOp();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        lm.setPower(gamepad1.left_stick_y);
        rm.setPower(gamepad1.right_stick_y);
        if(gamepad1.left_bumper)
            bm.setPower(-1);
        else if(gamepad1.right_bumper)
            bm.setPower(1);
        else
            bm.setPower(0);

        /*if(gamepad1.y)
            fm.setPower(-1);
        else if(gamepad1.b)
            fm.setPower(-0.75);
        else if(gamepad1.a)
            fm.setPower(0);*/
    }

    @Override
    public void stop() {
        lm.setZeroPowerBehavior(FLOAT);
        rm.setZeroPowerBehavior(FLOAT);
    }

    public void waitSec(double seconds) {
        ElapsedTime waitSec = new ElapsedTime();
        waitSec.reset();
        while (waitSec.seconds() < seconds) {
        }
    }

    public void hardwareMWTeleOp() {

        lm = hardwareMap.dcMotor.get("lm");
        rm = hardwareMap.dcMotor.get("rm");
        //fm = hardwareMap.dcMotor.get("fm");
        bm = hardwareMap.dcMotor.get("bm");
        //rm.setDirection(DcMotorSimple.Direction.REVERSE);
        //fs = hardwareMap.servo.get("fs");

        lm.setDirection(DcMotorSimple.Direction.REVERSE);
        rm.setDirection(DcMotorSimple.Direction.REVERSE);

        lm.setZeroPowerBehavior(BRAKE);
        rm.setZeroPowerBehavior(BRAKE);
    }


    /*public void getHeadingTelemetry() {
        Telemetry headingTelemetry = new TelemetryImpl(this);
        headingTelemetry.addData("Heading", AngleUnit.DEGREES.normalize(angles.firstAngle));
        updateTelemetry(headingTelemetry);
    }*/

    /*public float getHeading() {
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        return AngleUnit.DEGREES.normalize(angles.firstAngle);
    }*/


    public boolean inRange(double n1, double n2, double range) {
        if(Math.abs(n1-n2) <= range) return true;
        else return false;
    }

    public void runMotors(DcMotor[] motors, double power) {
        for(int i=0; i<motors.length; i++) {
            motors[i].setPower(power);
        }
    }




    public double sinDeg(double degrees) {
        return Math.sin(Math.toRadians(degrees));
    }

    public double cosDeg(double degrees) {
        return Math.cos(Math.toRadians(degrees));
    }

    public double atanDeg(double ratio) {
        return Math.toDegrees(Math.atan(ratio));
    }

    public void brakeMotors() {
        lm.setZeroPowerBehavior(BRAKE);
        rm.setZeroPowerBehavior(BRAKE);
    }

    public void coastMotors() {
        lm.setZeroPowerBehavior(FLOAT);
        rm.setZeroPowerBehavior(FLOAT);
    }

    public float round(float n, int places) {
        return (float) (Math.round(n * Math.pow(10, places)) / Math.pow(10, places));
    }

    public double round(double n, int places) {
        return (Math.round(n * Math.pow(10, places)) / Math.pow(10, places));
    }

    public int is(Boolean bool) {
        return (bool ? 1 : 0);
    }
    public int isnt(Boolean bool) {
        return (is(bool) == 1 ? 0 : 1);
    }
    public int isNot(Boolean bool) {
        return isnt(bool);
    }
    public int isNeg(Boolean bool) {
        return (bool ? 1 : -1);
    }
}
