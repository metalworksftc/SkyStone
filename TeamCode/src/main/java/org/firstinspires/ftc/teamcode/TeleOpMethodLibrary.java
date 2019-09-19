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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;

import java.util.ArrayList;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

@Disabled
public class TeleOpMethodLibrary extends OpMode {

    ElapsedTime driveModeChanger = new ElapsedTime();
    String driveMode = "mecanum";
    int count = 0;
    double angle = 0;
    double magnitude = 0;
    double stickY = 0;
    double stickX = 0;
    boolean strafing = false;
    ArrayList<Integer> counters = new ArrayList<Integer>(0);
    DcMotor leftFrontM, rightFrontM, leftBackM, rightBackM;
    DcMotor liftM, pivotM, slideM, beaterBarM;
    DcMotor[] drive = {leftFrontM, leftBackM, rightFrontM, rightBackM};
    DcMotor[] all;
    final double OPEN_POSITION = 1; //0
    final double CLOSED_POSITION = 0; //1

    Servo storageServo;
    Servo markerServo;

    DigitalChannel armLimit;

    double lFPow, rFPow, lBPow, rBPow;
    double powerMultiplier = 1, turningSpeed = 0;
    final double FULL_POWER = 1, SLOW_POWER = 0.6, STRAFING_POWER = 1, TURNING_POWER = 0.85;

    boolean acceptCubes = true;

    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime timeoutCheck = new ElapsedTime();
    BNO055IMU imu;
    Orientation angles = new Orientation();
    boolean cheezyToggle = true;
    int hold = 0;
    float hsvValues[] = {0F, 0F, 0F};
    final float values[] = hsvValues;
    final double SCALE_FACTOR = 255;
    int relativeLayoutId;

    public final double MINERAL_SERVO_DOWN = 1;
    public final double MINERAL_SERVO_UP = -1;
    public final double MINERAL_SERVO_MID = 0.5;


    double powerMultP = 0.5, powerMultS = 1;
    double encoderSet = 0;

    @Override
    public void init() {

    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
    }

    @Override
    public void stop() {
    }

    public void waitSec(double seconds) {
        ElapsedTime waitSec = new ElapsedTime();
        waitSec.reset();
        while (waitSec.seconds() < seconds) {
        }
    }

    public void hardwareMWTeleOp() {

        leftFrontM = hardwareMap.dcMotor.get("lfm");
        leftBackM = hardwareMap.dcMotor.get("lbm");
        rightFrontM = hardwareMap.dcMotor.get("rfm");
        rightBackM = hardwareMap.dcMotor.get("rbm");

        leftFrontM.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackM.setDirection(DcMotorSimple.Direction.REVERSE);

        leftBackM.setZeroPowerBehavior(BRAKE);
        rightBackM.setZeroPowerBehavior(BRAKE);
        leftFrontM.setZeroPowerBehavior(FLOAT);
        rightFrontM.setZeroPowerBehavior(FLOAT);

        leftFrontM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftM = hardwareMap.dcMotor.get("lm");
        //liftM.setDirection(DcMotorSimple.Direction.REVERSE);
        liftM.setZeroPowerBehavior(BRAKE);

        pivotM = hardwareMap.dcMotor.get("pm");
        //pivotM.setDirection(DcMotorSimple.Direction.REVERSE);
        pivotM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivotM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivotM.setZeroPowerBehavior(BRAKE);

        slideM = hardwareMap.dcMotor.get("sm");
        slideM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideM.setZeroPowerBehavior(BRAKE);

        beaterBarM = hardwareMap.dcMotor.get("bbm");
        beaterBarM.setZeroPowerBehavior(BRAKE);

        all = new DcMotor[]{leftFrontM, leftBackM, rightFrontM, rightBackM, liftM, pivotM, slideM};
        drive = new DcMotor[]{leftFrontM, leftBackM, rightFrontM, rightBackM};

        markerServo = hardwareMap.servo.get("ms");
        storageServo = hardwareMap.servo.get("bs");

        storageServo.setPosition(CLOSED_POSITION);

        armLimit = hardwareMap.digitalChannel.get("al");
        armLimit.setMode(DigitalChannel.Mode.INPUT);

        //Gyro Init
        BNO055IMU.Parameters parameters2 = new BNO055IMU.Parameters();
        parameters2.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters2.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters2.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters2.loggingEnabled = true;
        parameters2.loggingTag = "IMU";
        parameters2.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters2);

        //Color Sensor Init
        relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());

    }

    public void hardwareMWTesting() {

        leftFrontM = hardwareMap.dcMotor.get("lfm");
        leftBackM = hardwareMap.dcMotor.get("lbm");
        rightFrontM = hardwareMap.dcMotor.get("rfm");
        rightBackM = hardwareMap.dcMotor.get("rbm");

        leftFrontM.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackM.setDirection(DcMotorSimple.Direction.REVERSE);

        leftBackM.setZeroPowerBehavior(BRAKE);
        rightBackM.setZeroPowerBehavior(BRAKE);
        leftFrontM.setZeroPowerBehavior(BRAKE);
        rightFrontM.setZeroPowerBehavior(BRAKE);

        leftFrontM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        all = new DcMotor[]{leftFrontM, leftBackM, rightFrontM, rightBackM, liftM, pivotM, slideM};
        drive = new DcMotor[]{leftFrontM, leftBackM, rightFrontM, rightBackM};

        BNO055IMU.Parameters parameters2 = new BNO055IMU.Parameters();
        parameters2.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters2.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters2.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters2.loggingEnabled = true;
        parameters2.loggingTag = "IMU";
        parameters2.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters2);
    }


    public void activateGyro() {
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }

    public void getHeadingTelemetry() {
        Telemetry headingTelemetry = new TelemetryImpl(this);
        headingTelemetry.addData("Heading", AngleUnit.DEGREES.normalize(angles.firstAngle));
        updateTelemetry(headingTelemetry);
    }

    public float getHeading() {
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        return AngleUnit.DEGREES.normalize(angles.firstAngle);
    }


    public boolean inRange(double n1, double n2, double range) {
        if (Math.abs(n1 - n2) <= range) return true;
        else return false;
    }

    public void runMotors(DcMotor[] motors, double power) {
        for (int i = 0; i < motors.length; i++) {
            motors[i].setPower(power);
        }
    }

    boolean isControlling = false;
    int encoderSetSlide = 0;
    int previousEncoderSlide = 0;
    double powerMultTimeOutS = 1;

    boolean toOpen = false;
    boolean toOpenForFun = false;
    boolean wasControlling = false;

    public void controlArm() {

        //Down is down into the ground when collecting
        if (Math.abs(gamepad2.right_stick_y) > 0.1) {
            if (armLimit.getState()) {
                if (!gamepad2.dpad_up)
                    pivotM.setPower(-gamepad2.right_stick_y * powerMultP);
                else
                    pivotM.setPower(-gamepad2.right_stick_y * powerMultP * 0.5);
            } else {
                if (-gamepad2.right_stick_y > 0)
                    pivotM.setPower(-gamepad2.right_stick_y * powerMultP);
                else
                    pivotM.setPower(0);
            }
            encoderSet = pivotM.getCurrentPosition();
        } else if (Math.abs(pivotM.getCurrentPosition() - encoderSet) > 2) {
            pivotM.setPower((encoderSet - pivotM.getCurrentPosition()) * 0.001);
        }

        if (timeoutCheck.seconds() > 1) {
            timeoutCheck.reset();
            if (Math.abs(gamepad2.left_stick_y) > 0.1) {
                if (Math.abs(encoderSetSlide - previousEncoderSlide) > 10)
                    powerMultTimeOutS = 1;
                else
                    powerMultTimeOutS = 0.2;
                previousEncoderSlide = encoderSetSlide;
            }
        }

        //Down is in
        if (Math.abs(gamepad2.left_stick_y) > 0.1) {
            encoderSetSlide = slideM.getCurrentPosition();
            slideM.setPower(-gamepad2.left_stick_y * powerMultS * powerMultTimeOutS);
        } else if (Math.abs(slideM.getCurrentPosition() - encoderSetSlide) > 2 && !isControlling) {
            slideM.setPower((encoderSetSlide - slideM.getCurrentPosition()) * 0.0015);
            powerMultTimeOutS = 1;
        }

        /*if(isControlling && Math.abs(gamepad2.left_stick_y) < 0.1) {
            slideM.setPower(0.1);
        }*/

        beaterBarM.setPower(are(gamepad2.right_bumper, gamepad2.right_trigger > 0));

        if (gamepad2.left_trigger != 0) {
            storageServo.setPosition(OPEN_POSITION);
            beaterBarM.setPower(-1);
            runtime.reset();
            toOpen = true;
        }
        if (toOpen && runtime.seconds() >= 1.25) {
            toOpen = false;
            beaterBarM.setPower(0);
            storageServo.setPosition(CLOSED_POSITION);
        }
        if (gamepad2.left_bumper) {
            storageServo.setPosition(OPEN_POSITION); //0.4
            beaterBarM.setPower(-1);
            runtime.reset();
            toOpen = true;
        }
        if (toOpen && runtime.seconds() >= 1.25) {
            toOpen = false;
            beaterBarM.setPower(0);
            storageServo.setPosition(CLOSED_POSITION);
        }



      /*if(gamepad2.dpad_left) {
            storageServo.setPosition(OPEN_POSITION);
            beaterBarM.setPower(-1);
            runtime.reset();
            toOpenForFun = true;
      }
      if(toOpenForFun && runtime.seconds() >= 0.5) {
            storageServo.setPosition(CLOSED_POSITION);
            toOpenForFun = false;
            beaterBarM.setPower(0);
      }*/


        telemetry.addLine("ServoPos: " + storageServo.getPosition());
        telemetry.addLine("EncoderSet: " + encoderSet);
        telemetry.addLine("Pivot Current Position: " + pivotM.getCurrentPosition());
        telemetry.addLine("EncoderSetSlide: " + encoderSetSlide);
        telemetry.addLine("Slide Current Position: " + slideM.getCurrentPosition());
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
        leftFrontM.setZeroPowerBehavior(BRAKE);
        leftBackM.setZeroPowerBehavior(BRAKE);
        rightFrontM.setZeroPowerBehavior(BRAKE);
        rightBackM.setZeroPowerBehavior(BRAKE);
        liftM.setZeroPowerBehavior(BRAKE);
    }

    public void coastMotors() {
        leftFrontM.setZeroPowerBehavior(FLOAT);
        leftBackM.setZeroPowerBehavior(FLOAT);
        rightFrontM.setZeroPowerBehavior(FLOAT);
        rightBackM.setZeroPowerBehavior(FLOAT);
        liftM.setZeroPowerBehavior(BRAKE);
    }

    public float round(float n, int places) {
        return (float) (Math.round(n * Math.pow(10, places)) / Math.pow(10, places));
    }

    public double round(double n, int places) {
        return (Math.round(n * Math.pow(10, places)) / Math.pow(10, places));
    }

    public float pow(float n, double power) {
        return (float) Math.pow(n, power);
    }

    public double pow(double n, double power) {
        return Math.pow(n, power);
    }

    public int b1Count = 0;
    public boolean hangDrive = false;

    public void hangDrive() {
        if (b1Count < 30) {
            b1Count++;
        } else if (gamepad1.b) {
            b1Count = 0;
            hangDrive = !hangDrive;
            slowMode = hangDrive;
        }
    }

    public double pivotPowerScale() {
        double power = 0;
        double degrees = 0;
        final double ENCODER_COUNTS_PER_DEGREE = 4.66667;
        degrees = pivotM.getCurrentPosition() * ENCODER_COUNTS_PER_DEGREE;
        if (degrees >= 90 && gamepad2.left_stick_y < 0) power = -0.2;
        else if (degrees <= 90 && gamepad2.left_stick_y > 0) power = 0.2;
        else if (gamepad2.left_stick_y < 0) power = clipToRange(-1 * sinDeg(degrees) + 0, -1, -0.1);
        else if (gamepad2.left_stick_y > 0) power = clipToRange(1 * sinDeg(degrees) + 0, 0.1, 1);
        else power = 0;
        return power;
    }

    public boolean slowMode = false;
    int y1Count = 0;

    public void slowMode() {
        if (y1Count < 30) {
            y1Count++;
        } else if (gamepad1.y) {
            y1Count = 0;
            slowMode = !slowMode;
        }
        //slowMode = toggleWith(gamepad1.y, 0, slowMode);
    }

    double storagePos = 0;

    public final double PIVOT_COEFFICIENT = 0;
    public final double PIVOT_CONSTANT = 0;

    /*public boolean toggleWith(boolean button, int index, boolean isToggled) {
        if(counters.size() < index) counters.add(index, 0);
        if(counters.get(index) < 30) {
            counters.set(index, counters.get(index)+1);
        } else if(button) {
            counters.set(index, 0);
            isToggled = !isToggled;
        }
        return isToggled;
    }*/

    /*public float getHue() {
        Color.RGBToHSV((int) colorSensor.red(), (int) colorSensor.green(), (int) colorSensor.blue(), hsvValues);
        return hsvValues[0];
    }*/

    public void tankDrive() {

        stickY = gamepad1.left_stick_y; //0
        stickX = -gamepad1.left_stick_x * 0.5;//1

        if (gamepad1.right_trigger != 0) {
            stickX = -Math.abs(gamepad1.right_trigger);
            strafing = true;
        } else if (gamepad1.left_trigger != 0) {
            stickX = Math.abs(gamepad1.left_trigger);
            strafing = true;
        } else strafing = false;

        if (stickY != 0) angle = atanDeg(stickX / stickY);
        else angle = 90 + 180 * is(stickX < 0);
        if (stickY < 0) angle += 180;
        if (angle < 0) angle += 360;
        magnitude = Math.sqrt(stickX * stickX + stickY * stickY);
        if (!isControlling) turningSpeed = TURNING_POWER * gamepad1.right_stick_x;
        if (gamepad1.dpad_left) {
            turningSpeed = -0.25;
        } else if (gamepad1.dpad_right) {
            turningSpeed = 0.25;
        }
        operatorTurning();

        rFPow = magnitude * cosDeg(angle + 45 - 90 * is(hangDrive)) + turningSpeed;
        lFPow = magnitude * sinDeg(angle + 45 - 90 * is(hangDrive)) - turningSpeed;
        rBPow = magnitude * sinDeg(angle + 45 - 90 * is(hangDrive)) + turningSpeed;
        lBPow = magnitude * cosDeg(angle + 45 - 90 * is(hangDrive)) - turningSpeed;

        double max = Math.max(Math.max(Math.abs(lFPow), Math.abs(lBPow)), Math.max(Math.abs(rFPow), Math.abs(rBPow)));
        if (max > 1) {
            lFPow /= max;
            lBPow /= max;
            rFPow /= max;
            rBPow /= max;
        }

        lBPow += 0.2 * isNeg(lBPow > 0) * is(strafing && slowMode);
        lFPow += 0.2 * isNeg(lFPow > 0) * is(strafing && slowMode);
        rBPow += 0.2 * isNeg(rBPow > 0) * is(strafing && slowMode);
        rFPow += 0.2 * isNeg(rFPow > 0) * is(strafing && slowMode);

        powerMultiplier = FULL_POWER * is(!slowMode) + SLOW_POWER * is(slowMode);

        leftBackM.setPower(powerMultiplier * lBPow);
        leftFrontM.setPower(powerMultiplier * lFPow);
        rightBackM.setPower(powerMultiplier * rBPow);
        rightFrontM.setPower(powerMultiplier * rFPow);
    }

    public void absoluteDrive(int offset) {
        double[] powers = getAbsPowers(gamepad1.left_stick_x, -gamepad1.left_stick_y, offset);
        for (int i = 0; i < 4; i++) {
            drive[i].setPower(powers[i]);
        }
        operatorTurning();
    }

    public boolean isTank = true;
    public int tankCount = 0;

    public void toggleDrive(int offset) {
        tankCount += is(gamepad1.a);
        tankCount = tankCount * is(gamepad1.a);
        if (tankCount > 80) {
            tankCount = 0;
            isTank = !isTank;
        }
        if (isTank) tankDrive();
        else absoluteDrive(offset);
        telemetry.addLine((isTank ? "Drive Mode: Tank" : "Drive Mode: Absolute"));
        telemetry.addLine("Tank Count: " + tankCount);
    }

    public double[] getAbsPowers(double x, double y, int offset) {
        double magnitude, angle;
        double[] vector = getVector(x, y, offset);
        magnitude = vector[0];
        angle = vector[1];
        double dist = getHeading() - angle;
        if (dist < 0) dist += 360;
        double lf, lb, rf, rb;
        if(!isControlling) turningSpeed = TURNING_POWER * gamepad1.right_stick_x;
        else turningSpeed = 0;
        rf = magnitude * cosDeg(dist + 45) + turningSpeed;
        lf = magnitude * sinDeg(dist + 45) - turningSpeed;
        rb = magnitude * sinDeg(dist + 45) + turningSpeed;
        lb = magnitude * cosDeg(dist + 45) - turningSpeed;
        double max = Math.max(Math.max(Math.abs(lFPow), Math.abs(lBPow)), Math.max(Math.abs(rFPow), Math.abs(rBPow)));
        if (max > 1) {
            lf /= max;
            lb /= max;
            rf /= max;
            rb /= max;
        }
        return new double[]{lf, lb, rf, rb};
    }

    public double[] getVector(double x, double y, int offset) {
        double magnitude, angle;
        magnitude = Math.sqrt(exp(x, 2) + exp(y, 2));
        if (y > 0) angle = -atanDeg(x / y);
        else if (y < 0) angle = -atanDeg(x / y) + 180;
        else angle = 90 * isNeg(x < 0);
        angle = angle - 360 * is(angle > 180);
        return new double[]{magnitude, angle + offset};
    }

    public double exp(double base, double power) {
        return Math.pow(base, power);
    }

    public int is(boolean bool) {
        return (bool ? 1 : 0);
    } //The is() method outputs 1 if the boolean (true/false) input is true and 0 if it is false.
    //We use this most often to replace very short if-else statements.

    public int isNeg(boolean bool) {
        return (bool ? 1 : -1);
    } //The isNeg() method does the same thing as the is() method, but if the boolean is false, it outputs -1 instead of 0.
    //This is most useful for setting motor powers and the like.

    public int are(boolean bool, boolean bool2) {
        return is(!(bool == bool2)) * isNeg(bool);
    } //The are() method accepts 2 booleans instead of just one. It is less versatile than the is() method, but much more
    //streamlined for certain usages. The first use of this is for the lift motor. Here's a map of the possible outputs:
    //bool & bool2 are both true: return 0
    //bool & bool2 are both false: return 0
    //bool is true, bool2 is false: return 1

    /*public boolean isCube() {
        Color.RGBToHSV((int) (sortingSensor.red() * SCALE_FACTOR),
                (int) (sortingSensor.green() * SCALE_FACTOR),
                (int) (sortingSensor.blue() * SCALE_FACTOR),
                hsvValues);
        return (hsvValues[1] > 0.3);
    }*/

    public void testSpeed() {
        double leftPower = -gamepad1.left_stick_y;
        double rightPower = -gamepad1.right_stick_y;
        leftFrontM.setPower(leftPower);
        leftBackM.setPower(leftPower);
        rightFrontM.setPower(rightPower);
        rightBackM.setPower(rightPower);
    }

    /*public void sortMineral() {
        //Stop the sorting motor
        if(acceptCubes) {
            if(isCube()) {
                //Power servo to kick it into the bucket
            } else {
                //Do nothing, mineral continues to run off of the robot
            }
        } else {
            if(isCube()) {
                //Do nothing, mineral continues to run off of the robot
            } else {
                //Power servo to kick it into the bucket
            }
        }
    }*/

    public int pivotStartingEncoderCount, slideStartingEncoderCount;
    public double brakeSum;
    public int priorError;

    public void setStartingEncoderCount(DcMotor motor, int position) {
        brakeSum = 0;
        pivotStartingEncoderCount = position;
        motor.setZeroPowerBehavior(BRAKE);
        priorError = 0;
    }

    public double PBrake = 0.0025;
    public double IBrake = 0;
    public double DBrake = 0;
    int errorBrake = 0;

    public double clipToRange(double n, double lower, double upper) {
        return n * is(n > upper || n < lower) + lower * is(n < lower) + upper * is(n > upper);
    }

    public float clipToRange(float n, float lower, float upper) {
        return n * is(n > upper || n < lower) + lower * is(n < lower) + upper * is(n > upper);
    }

    public int clipToRange(int n, int lower, int upper) {
        return n * is(n > upper || n < lower) + lower * is(n < lower) + upper * is(n > upper);
    }

    public void hardBrakePivot(DcMotor motor) {
        priorError = errorBrake;
        errorBrake = motor.getCurrentPosition() - pivotStartingEncoderCount;
        brakeSum = 0.9 * brakeSum + errorBrake;
        double speed = -(PBrake * errorBrake + IBrake * brakeSum + DBrake * (errorBrake - priorError));
        motor.setPower(speed);
        /*telemetry.addLine("Start: " + pivotStartingEncoderCount);
        telemetry.addLine("Current: " + motor.getCurrentPosition());
        telemetry.addLine("Error: " + errorBrake);
        telemetry.addLine("Sum: "+brakeSum);
        telemetry.addLine("Speed: " + speed);
        telemetry.update();
        //liftM.setPower(0.2);
        */
    }

    int step = 0;
    boolean isExecuting = false;
    boolean ranAgain = false;

    public void scoreMinerals() {
        /* Servo open
         * Beaterbar to collection speed
         * Delay
         * Servo closes
         * Pivot goes down
         * Small delay
         * Retract arm
         */
        if (gamepad2.a) {
            isControlling = true;
            if (gamepad2.left_bumper) {
                isExecuting = true;
                switch (step) {
                    case 0: {
                        storageServo.setPosition(OPEN_POSITION);
                        beaterBarM.setPower(-1);
                        step++;
                    }
                    case 1: {
                        runtime.reset();
                        step++;
                    }
                    case 2: {
                        if (runtime.seconds() >= 0.3) {
                            storageServo.setPosition(CLOSED_POSITION);
                            step++;
                        }
                    }
                    case 3: {
                        pivotM.setPower(-powerMultP);
                        isControlling = true;
                        encoderSet = 0;
                        step++;
                    }
                    case 4: {
                        runtime.reset();
                        step++;
                    }
                    case 5: {
                        if (runtime.seconds() >= 0.5) {
                            slideM.setPower(powerMultS);
                            step++;
                        }
                    }
                    case 6: {
                        runtime.reset();
                        step++;
                    }
                    case 7: {
                        if (runtime.seconds() >= 0.5) {
                            isControlling = false;
                            slideM.setPower(0);
                        }
                        isExecuting = false;
                    }
                }
            }
        } else {
            step = 0;
            isControlling = isExecuting;
        }
    }

    public void operatorTurning() {
        if (Math.abs(gamepad2.right_stick_x) > 0.9) {
            turningSpeed = 0.125 * are(gamepad2.right_stick_x > 0.9, gamepad2.right_stick_x < -0.9);
            isControlling = true;
        } else isControlling = false;
    }

}
