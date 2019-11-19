package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public abstract class MethodLibrary extends LinearOpMode {

    /*
     * Instance Variables
     */

    // Motors
    //protected DcMotor leftFrontM, leftBackM, rightFrontM, rightBackM;
    protected DcMotor left, right, horizontal, vertical;

    // Gyro Sensor instance variables
    protected BNO055IMU imu;
    protected int relativeLayoutId;
    protected Orientation angles;
    //color sensor
    protected ColorSensor cs;
   // Servo
    protected Servo tailServo, stoneServo;
    protected OpticalDistanceSensor testsensor;




    /*
     * Methods
     */

    protected void waitSec(double seconds) {

        ElapsedTime time = new ElapsedTime();
        time.startTime();

        while(time.seconds() < seconds && opModeIsActive()) {
            telemetry.addLine("Waiting for " + (seconds - time.seconds()) + " seconds");
            telemetry.update();
        }

    }

    /**
     * @return the angle (in degrees) the robot is currently facing towards, between -180 to 180 exclusive
     */
    protected float getHeading() {
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        return AngleUnit.DEGREES.normalize(angles.firstAngle);
    }


    // Initialize the Autonomous hardware map
    protected void hardwareMap() {

        left = hardwareMap.dcMotor.get("lm");
        right = hardwareMap.dcMotor.get("rm");
        vertical = hardwareMap.dcMotor.get("bm");
        tailServo = hardwareMap.servo.get("ts");
        stoneServo = hardwareMap.servo.get("ss");
        vertical =  hardwareMap.dcMotor.get("tm");
        horizontal = hardwareMap.dcMotor.get("bm");
//        testsensor = hardwareMap.opticalDistanceSensor.get("tds");

        left.setDirection(DcMotorSimple.Direction.REVERSE);
        horizontal.setDirection(DcMotorSimple.Direction.REVERSE);

        BNO055IMU.Parameters parameters2 = new BNO055IMU.Parameters();
        parameters2.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters2.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters2.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters2.loggingEnabled      = true;
        parameters2.loggingTag          = "IMU";
        parameters2.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters2);
        //activateGyro(); No longer used by us.
        relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        telemetry.addLine("Completed Gyro Init");

        telemetry.addLine("Status: Initialized");
        telemetry.update();

        cs = hardwareMap.colorSensor.get("cs");



    }

    /**
     * Drives forwards only
     * @param inches the number of inches to drive
     * @param power the speed at which to drive, between -1 and 1 inclusive
     */
    protected void drive(double inches, double power) {

        final double COUNTS_PER_INCH = 1000.0/13.50;

        int target = left.getCurrentPosition() + (int) (COUNTS_PER_INCH * inches);

        left.setPower(power);
        right.setPower(power);

        while (left.getCurrentPosition() < target) {
            telemetry.addLine("Driving: " + left.getCurrentPosition() + " of " + target + " counts");
            telemetry.update();
        }

        left.setPower(0);
        right.setPower(0);
        waitSec(.5);
    }


    /**
     * Drives forwards only
     * @param inches the number of inches to drive
     * @param power the speed at which to drive, between -1 and 1 inclusive
     */
    protected void reverse(double inches, double power) {

        final double COUNTS_PER_INCH = -1000.0/13.50;

        power = -power;

        int target = left.getCurrentPosition() + (int) (COUNTS_PER_INCH * inches);

        left.setPower(power);
        right.setPower(power);

        while (left.getCurrentPosition() > target) {
            telemetry.addLine("Driving: " + left.getCurrentPosition() + " of " + target + " counts");
            telemetry.update();
        }


        left.setPower(0);
        right.setPower(0);
        waitSec(0.5);
    }

    protected void extend(double inches) {

        final double COUNTS_PER_INCH = 1000.0/7.0;

        int target = horizontal.getCurrentPosition() + (int) (COUNTS_PER_INCH * inches);

        horizontal.setPower(0.5);

        while (horizontal.getCurrentPosition() < target) {
            telemetry.addLine("Driving: " + horizontal.getCurrentPosition() + " of " + target + " counts");
            telemetry.update();
        }

        horizontal.setPower(0);
        waitSec(.5);
    }

    /**
     * Drives forwards only
     * @param inches the number of inches to drive
     */
    protected void retract (double inches) {

        final double COUNTS_PER_INCH = -1000.0/7.0;

        int target = horizontal.getCurrentPosition() + (int) (COUNTS_PER_INCH * inches);

        horizontal.setPower(-0.5);

        while (horizontal.getCurrentPosition() > target) {
            telemetry.addLine("Driving: " + horizontal.getCurrentPosition() + " of " + target + " counts");
            telemetry.update();
        }


        horizontal.setPower(0);
        waitSec(0.5);
    }

    protected void raise(double inches) {

        final double COUNTS_PER_INCH = 1000.0/1.875;

        int target = vertical.getCurrentPosition() + (int) (COUNTS_PER_INCH * inches);

        vertical.setPower(0.5);

        while (vertical.getCurrentPosition() < target) {
            telemetry.addLine("Driving: " + vertical.getCurrentPosition() + " of " + target + " counts");
            telemetry.update();
        }

        vertical.setPower(0);
        waitSec(.5);
    }

    /**
     * Drives forwards only
     * @param inches the number of inches to drive
     */
    protected void lower(double inches) {

        final double COUNTS_PER_INCH = -1000.0/1.875;

        int target = vertical.getCurrentPosition() + (int) (COUNTS_PER_INCH * inches);

        vertical.setPower(-0.5);

        while (vertical.getCurrentPosition() > target) {
            telemetry.addLine("Driving: " + vertical.getCurrentPosition() + " of " + target + " counts");
            telemetry.update();
        }


        vertical.setPower(0);
        waitSec(0.5);
    }
    /**
     * You have to write this!
     * @param target the angle (in degrees) to turn to, between -180 and 180 inclusive
     */
    protected void absoluteTurn(float target) {
        //turn left
        float distLeft = target - getHeading();
        if (distLeft < 0){
            distLeft += 360;
        }
        float distRight =  360 -  distLeft;

        if (distLeft < distRight) {
            //turn left
            while (distLeft > 8) {
                distLeft = target - getHeading();
                if (distLeft < 0){
                    distLeft += 360;
                }
//                double power = 0.01* (distLeft + 20);
////                if (power > 0.6) {
////                    power = 0.6;
////                }
                double power = 0.7;
                left.setPower(-power);
                right.setPower(power);
            }
        } else {
            //turn right
            while (distRight > 8) {
                distLeft = target - getHeading();
                if (distLeft < 0){
                    distLeft += 360;
                }
                distRight = 360 - distLeft;
//                double power = 0.01* (distRight + 20);
//                if (power > 0.6) {
//                    power = 0.6;
//                }
                double power = 0.7;
                left.setPower(power);
                right.setPower(-power);
            }
        }
        left.setPower(0);
        right.setPower(0);
        waitSec(0.5);
    }

    protected void engageHook() {
        tailServo.setPosition(0.6);
        waitSec(1);
    }

    protected void disengageHook() {
        tailServo.setPosition(1);
        waitSec(1);
    }

    protected void grab()  {
        stoneServo.setPosition(1);
        waitSec(1.5);
    }
    protected void release() {
        stoneServo.setPosition(0.5);
        waitSec(1.5);
    }


    public void pickUpBlock() {
        raise(4.5);
        extend(7.5);
        lower(4.5);
        grab();
    }


    public void deliverBlock(double dist, String dir) {
        drive(5,0.5);
        if (dir ==  "l") absoluteTurn(-90);
        if (dir == "r") absoluteTurn(90);
        reverse(dist,0.8);
        //parks on line
        drive(20,0.7);
    }

}
