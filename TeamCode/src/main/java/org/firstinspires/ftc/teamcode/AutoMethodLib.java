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

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;

import java.util.List;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark.UNKNOWN;

//C:\Users\metal\ftc_app_17_9_12\FtcRobotController\build\intermediates\res\merged\release\raw

@Disabled
public class AutoMethodLib extends LinearOpMode {

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    public static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    public static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AYc5tcP/////AAAAGSEDvoD9+UpNiaRl0pLjSVwXxvxIMzbg7jm0YHRpIEHEncuVzyDs87PTMXmHBy6Hu7zrsk1lZn1llT5ifcdzFsH9SeC/JE3PN1GoDwHeYFMcNQTgQUjRE9KIW2TqIa/rpbguzNC6YAL7miJ6FALHxWtDukHtY4qZmcVSr6QfxD9Gx9YyA9yQ2mvMeKJqHCvMJq2iztwJgBVOGlgf/ufkCW3yz8FnT+CHWguHrhEM4yupxpF6jz+h5ZBXC0gvePP3F35YD36aaoLmwvbpl1f2SHrTqeN+WJ3eDlYOrzP69akMxzYJ97XUYr17ArRPwHvDF/pQOcbyeSiIjC6SUr+nh5Nyn4sM56gXC4bZZg9+Pbzh";

    String univVuMark = "CENTER";

    public final double COLOR_SERVO_DOWN = 1;
    public final double COLOR_SERVO_UP = -1;
    public final double MARKER_SERVO_DOWN = 1;
    public final double MARKER_SERVO_UP = 0.2;
    public final double MINERAL_SERVO_DOWN = 1;
    public final double MINERAL_SERVO_UP = -1;
    public final double MINERAL_SERVO_MID = 0.5;

    public static double P = 0.0075;
    public static double I = 0.00025;
    public static double D = 0.0144;

    public static double kP = 1/15;
    public static double kI = 0;
    public static double kD = 0;

    int goldDetected;

    public TFObjectDetector tfod;

    public List<Recognition> updatedRecognitions;

    /*
     * This section of the code works with declaring the different parts of our robot and code.
     * This OpMode is the Method Library for our programming robot, not our competition robot.
     */
    ElapsedTime runtime = new ElapsedTime();
    /* The above line declares our main runtime.
     * Runtime allows us to count the amount of time that an OpMode takes.
     * This is useful for autonomous and for having a timer on the Robot Controller during TeleOp.
     */
    Telemetry write = new TelemetryImpl(this);
    double powerMultiplier = 1;
    DcMotor leftFrontM, rightFrontM, leftBackM, rightBackM, liftM;
    DcMotor[] driveMotors = new DcMotor[4];
    Servo markerServo;

    double lFPow, rFPow, lBPow, rBPow;

    //Declare the only servo on our programming robot.
    final String TAG = "Vuforia VuMark Sample";
    //I honestly have no idea what this is used for. It was part of the example code's Vuforia OpMode, so we put it in here.
    OpenGLMatrix lastLocation = null;
    //See above documentation...
    /*
     * The next 4 lines of code deal with variables and constructors that are necessary to use the Vuforia code provided in the example code.
     */
    VuforiaLocalizer vuforia;
    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;
    RelicRecoveryVuMark vuMark;
    BNO055IMU imu;
    //Declare the gyro (imu stands for inertial measurement unit.
    Orientation angles = new Orientation();
    //Angles and gravity are pre-made variables from the Gyro code. Every team that uses the gyro uses them.
    Acceleration gravity;

    float idealHeading = 0;
    //idealHeading is used in our gyroDrive() methods to calculate the heading that we should be at vs. the one that we are actually at in order to correct our turns.
    float hsvValues[] = {0F, 0F, 0F};
    final float values[] = hsvValues;
    final double SCALE_FACTOR = 255;
    int relativeLayoutId;

    /** hardwareMW() is a condensed way of implementing our declarations into a HardwareMap.
     * This really is a completely amazing time-saver and every team should use this.
     */
    public void hardwareMW() {
        telemetry.addData("Status", "Initialized");
        leftFrontM = hardwareMap.dcMotor.get("lfm");
        leftBackM = hardwareMap.dcMotor.get("lbm");
        rightFrontM = hardwareMap.dcMotor.get("rfm");
        rightBackM = hardwareMap.dcMotor.get("rbm");
        liftM = hardwareMap.dcMotor.get("lm");

        leftFrontM.setDirection(REVERSE);
        leftBackM.setDirection(REVERSE);

        leftFrontM.setZeroPowerBehavior(BRAKE);
        leftBackM.setZeroPowerBehavior(BRAKE);
        rightFrontM.setZeroPowerBehavior(BRAKE);
        rightBackM.setZeroPowerBehavior(BRAKE);
        liftM.setZeroPowerBehavior(BRAKE);

        leftFrontM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        driveMotors[0] = leftFrontM;
        driveMotors[1] = leftBackM;
        driveMotors[2] = rightFrontM;
        driveMotors[3] = rightBackM;

        markerServo = hardwareMap.servo.get("ms");
        //mineralServo = hardwareMap.servo.get("mins");

        /*
         * The next section of code deals with initializing Vuforia.
         * All of this was taken directly from the example code, so that's where you can find it.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        initTfod();
        updatedRecognitions = tfod.getUpdatedRecognitions();

        //activateVuforia(); No longer used by us.
        /*
         * The next section of code deals with initializing the gyro.
         * Yet again, all of this code was taken directly from the example code.
         */
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
        telemetry.addLine("Completed Gyro Init.");
        telemetry.update();
    }

    /**
     * This was used only for testing of TenserFlow
     */
    public void hardwareTFODTesting() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        initTfod();
        updatedRecognitions = tfod.getUpdatedRecognitions();

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
        telemetry.addLine("Completed Gyro Init.");
        telemetry.update();
    }

    /**
     * Does exactly what it says
     * @param seconds
     * @param power pretty sure positive is up???
     */
    public void moveLiftMotor(double seconds, double power) {
        liftM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftM.setPower(power);
        waitSec(seconds);
        liftM.setPower(0);
    }

    /**
     * Drives the robot at a specified angle for a specified amount of time. Not sure if it really works, but in theory it should.
     * @param seconds
     * @param angleDeg
     * @param speed
     * Pretty sure this doesn't work
     */
    public void timeDrive(double seconds, int angleDeg, double speed) {
        motorRun(angleDeg, speed);
        waitSec(seconds);
        stopDrive();
    }

    //Gives us telemetry that we can use to decipher what Vuforia sees. We don't use Vuforia this year.
    public String getVuforiaTelemetry() {
        //The getVuforiaTelemetry() method is taken directly from the example code, and it does exactly what its name says.
        //TODO: Change all instances of "telemetry" to "vuforiaTelemetry".
        vuMark = RelicRecoveryVuMark.from(relicTemplate);
        if (vuMark != UNKNOWN) {

            telemetry.addData("VuMark", "%s visible", vuMark);

            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
            telemetry.addData("Pose", formatVuf(pose));

            /* We further illustrate how to decompose the pose into useful rotational and
             * translational components */
            if (pose != null) {
                VectorF trans = pose.getTranslation();
                Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                // Extract the X, Y, and Z components of the offset of the target relative to the robot
                double tX = trans.get(0);
                double tY = trans.get(1);
                double tZ = trans.get(2);

                // Extract the rotational components of the target relative to the robot
                double rX = rot.firstAngle;
                double rY = rot.secondAngle;
                double rZ = rot.thirdAngle;
            }
        }
        else {
            telemetry.addData("VuMark", "not visible");
        }
        return vuMark.toString();
    }
    /**
     * This is our newest version of the encoderTank method. It's more condensed, efficient, and accurate.
     * @param inches determined by testing and some light calculations.
     * @param speed
     */
    public void newEncoderTank(double inches, double speed) {
        final double COUNTS_PER_INCH = 41;
        double distance = inches;
        motorMode(driveMotors, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        boolean stop = false;
        int newTarget = (int) (-inches * COUNTS_PER_INCH);
        setDriveTarget(newTarget);
        motorMode(driveMotors, DcMotor.RunMode.RUN_TO_POSITION);
        powerMotors(driveMotors, speed);
        while(opModeIsActive() && !stop) {
            distance = inches - leftFrontM.getCurrentPosition()/COUNTS_PER_INCH;
            telemetry.addLine("Speed: "+speed+" | Actual speed: "+leftFrontM.getPower());
            telemetry.addLine("L: "+leftFrontM.getCurrentPosition()+" of "+newTarget);
            telemetry.addLine("R: "+rightFrontM.getCurrentPosition()+" of "+newTarget);
            telemetry.update();
            if(!leftFrontM.isBusy() || !rightFrontM.isBusy()) stop = true;
        }
        stopDrive();
        motorMode(driveMotors, DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //Older version. Too much outdated usage to deprecate.
    public void encoderTank(double inches, double inputSpeed) {

        /* This is our main drive method, which uses the encoders to precisely calculate distance.
         * There is a bunch of stuff that we could do a lot more efficiently, but meh.
         */
        final double COUNTS_PER_INCH = 95.9; //COUNTS_PER_INCH is a calculation of the number of encoder ticks for every inch.
        double distance = inches;
        double speed = 0.75*inputSpeed;
        motorMode(driveMotors, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Reset the current encoder counts to 0;
        boolean leftStop = false;
        boolean rightStop = false;
        //Declare 2 booleans that are used to end the while() loop lower down.
        int newLeftTarget = (int) (-inches * COUNTS_PER_INCH);  /*Converts the input of inches into encoder counts*/
        int newRightTarget = (int) (-inches * COUNTS_PER_INCH); /**/
        leftFrontM.setTargetPosition(newLeftTarget);
        leftBackM.setTargetPosition(newLeftTarget); //new
        rightFrontM.setTargetPosition(newRightTarget);
        rightBackM.setTargetPosition(newRightTarget); //new
        //Set a target distance for the motors using the pre-calculated int values.
        leftFrontM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //RUN_TO_POSITION sets the motors into the mode that allows them to move using the setTargetPosition().
        //Make sure to set the motors to RUN_TO_POSITION only AFTER they have a target.
        leftFrontM.setPower(speed);
        rightFrontM.setPower(speed);
        leftBackM.setPower(speed);
        rightBackM.setPower(speed);
        //Turn on the motors on so that they actually move. We forget that part a lot.
        while (opModeIsActive() & !leftStop & !rightStop) { //We use opModeIsActive() in all of our while() loops.
            //This prevents the Robot Controller app from crashing when your press the stop button
            distance = inches - leftFrontM.getCurrentPosition()/COUNTS_PER_INCH;
            speed = inputSpeed;
            leftFrontM.setPower(speed);
            rightFrontM.setPower(speed);
            leftBackM.setPower(speed);
            rightBackM.setPower(speed);
            System.out.println("----------------------------------------");
            System.out.println(speed);

            telemetry.addLine("Speed: "+ leftFrontM.getPower());
            telemetry.addLine("Left: " + leftFrontM.getCurrentPosition() + " of " + leftFrontM.getTargetPosition());
            telemetry.addLine("Right: " + rightFrontM.getCurrentPosition() + " of " + rightFrontM.getTargetPosition());
            telemetry.update();
            //Add telemetry to give us the current and target positions of both motors.
            //TODO: Create a custom telemetry for this method.
            //TODO: Change all "[telemetryName].update;" to "updateTelemetry([telemetryName]);".
            if (!leftFrontM.isBusy() || !rightFrontM.isBusy() || !leftBackM.isBusy() || !rightBackM.isBusy()) { //isBusy() can only be used in RUN_TO_POSITION to check if the motor has reached their target.
                leftStop = true;
                rightStop = true;
            }
        }
        leftFrontM.setPower(0);
        rightFrontM.setPower(0);
        leftBackM.setPower(0);
        rightBackM.setPower(0);

        motorMode(driveMotors, DcMotor.RunMode.RUN_USING_ENCODER);

    }

    //This was only used once last year, but it's incredibly useful for programming in spots the robot can get stuck.
    public void encoderTankTimeout(double inches, double inputSpeed) {

        ElapsedTime driveTime = new ElapsedTime();
        driveTime.reset();
        int previousTicks = 0;
        int tps = 50;
        /* This is our main drive method, which uses the encoders to precisely calculate distance.
         * There is a bunch of stuff that we could do a lot more efficiently, but meh.
         */
        final double COUNTS_PER_INCH = 36.5; //COUNTS_PER_INCH is a calculation of the number of encoder ticks for every inch.
        double distance = inches;
        double speed = 0.75*inputSpeed;

        leftFrontM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Reset the current encoder counts to 0;
        boolean leftStop = false;
        boolean rightStop = false;
        //Declare 2 booleans that are used to end the while() loop lower down.
        int newLeftTarget = (int) (-inches * COUNTS_PER_INCH);  /*Converts the input of inches into encoder counts*/
        int newRightTarget = (int) (-inches * COUNTS_PER_INCH); /**/
        leftFrontM.setTargetPosition(newLeftTarget);
        rightFrontM.setTargetPosition(newRightTarget);
        leftBackM.setTargetPosition(newLeftTarget);
        rightBackM.setTargetPosition(newRightTarget);
        //Set a target distance for the motors using the pre-calculated int values.
        leftFrontM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //RUN_TO_POSITION sets the motors into the mode that allows them to move using the setTargetPosition().
        //Make sure to set the motors to RUN_TO_POSITION only AFTER they have a target.
        leftFrontM.setPower(speed);
        rightFrontM.setPower(speed);
        leftBackM.setPower(speed);
        rightBackM.setPower(speed);
        //Turn on the motors on so that they actually move. We forget that part a lot.
        while (opModeIsActive() & !leftStop & !rightStop ) { //We use opModeIsActive() in all of our while() loops.
            //This prevents the Robot Controller app from crashing when your press the stop button
            distance = inches - leftFrontM.getCurrentPosition()/COUNTS_PER_INCH;
            speed = inputSpeed;
            leftFrontM.setPower(speed);
            rightFrontM.setPower(speed);
            leftBackM.setPower(-.75*speed);
            rightBackM.setPower(-.75*speed);
            System.out.println("----------------------------------------");
            System.out.println(speed);

            if(driveTime.seconds() >= 1) {
                tps = Math.abs(leftFrontM.getCurrentPosition()) - Math.abs(previousTicks);
                previousTicks = leftFrontM.getCurrentPosition();
            }

            if(tps < 15) leftStop = true;

            telemetry.addLine("Speed: "+(speed));
            telemetry.addLine("Left: " + leftFrontM.getCurrentPosition() + " of " + leftFrontM.getTargetPosition());
            telemetry.addLine("Right: " + rightFrontM.getCurrentPosition() + " of " + rightFrontM.getTargetPosition());
            telemetry.addLine("TPS " + tps);
            telemetry.addLine("PrevEnc " + previousTicks);
            telemetry.update();
            //Add telemetry to give us the current and target positions of both motors.
            //TODO: Create a custom telemetry for this method.
            //TODO: Change all "[telemetryName].update;" to "updateTelemetry([telemetryName]);".
            if (!leftFrontM.isBusy() || !rightFrontM.isBusy()) { //isBusy() can only be used in RUN_TO_POSITION to check if the motor has reached their target.
                leftStop = true;
                rightStop = true;
            }
        }
        leftFrontM.setPower(0);
        rightFrontM.setPower(0);
        leftBackM.setPower(0);
        rightBackM.setPower(0);

    }

    //Uhh.. I don't remember what this does, and I don't feel like reading through it.
    public void timeDriveTank(double power, double seconds) {
        leftFrontM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFrontM.setPower(-power);
        leftBackM.setPower(-power);
        rightFrontM.setPower(-power);
        rightBackM.setPower(-power);
        waitSec(seconds);
        stopDrive();
    }

    //These two are self explanatory. Instead of going straight, strafe left or right.
    //TODO: Fix the encoder counts per inch for strafing (*6 or so)
    public void encoderStrafeLeft(double inches, double inputSpeed) {
        //lf & rb -1, lb & rf 1

        final double COUNTS_PER_INCH = 48.5; //COUNTS_PER_INCH is a calculation of the number of encoder ticks for every inch.
        double distance = inches;
        double speed = inputSpeed;
        leftFrontM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Reset the current encoder counts to 0;
        boolean leftStop = false;
        boolean rightStop = false;
        //Declare 2 booleans that are used to end the while() loop lower down.
        int newLeftTarget = (int) (inches * COUNTS_PER_INCH);  /*Converts the input of inches into encoder counts*/
        int newRightTarget = (int) (inches * COUNTS_PER_INCH); /**/
        leftFrontM.setTargetPosition(newLeftTarget);
        rightFrontM.setTargetPosition(-newRightTarget);
        //Set a target distance for the motors using the pre-calculated int values.
        leftFrontM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //RUN_TO_POSITION sets the motors into the mode that allows them to move using the setTargetPosition().
        //Make sure to set the motors to RUN_TO_POSITION only AFTER they have a target.
        leftFrontM.setPower(inputSpeed);
        rightFrontM.setPower(-inputSpeed);
        leftBackM.setPower(-inputSpeed);
        rightBackM.setPower(inputSpeed);

        //Turn on the motors on so that they actually move. We forget that part a lot.
        while (opModeIsActive() & !leftStop & !rightStop) { //We use opModeIsActive() in all of our while() loops.
            //This prevents the Robot Controller app from crashing when your press the stop button
            distance = inches - leftFrontM.getCurrentPosition()/COUNTS_PER_INCH;
            leftFrontM.setPower(speed);
            rightFrontM.setPower(-speed);
            leftBackM.setPower(-speed);
            rightBackM.setPower(speed);
            System.out.println("----------------------------------------");
            System.out.println(speed);

            telemetry.addLine("Speed: "+(speed));
            telemetry.addLine("Left: " + leftFrontM.getCurrentPosition() + " of " + leftFrontM.getTargetPosition());
            telemetry.addLine("Right: " + rightFrontM.getCurrentPosition() + " of " + rightFrontM.getTargetPosition());
            telemetry.update();
            //Add telemetry to give us the current and target positions of both motors.
            //TODO: Create a custom telemetry for this method.
            //TODO: Change all "[telemetryName].update;" to "updateTelemetry([telemetryName]);".

            if (!leftFrontM.isBusy() || !rightFrontM.isBusy()) { //isBusy() can only be used in RUN_TO_POSITION to check if the motor has reached their target.
                leftStop = true;
                rightStop = true;
            }
        }
        leftFrontM.setPower(0);
        rightFrontM.setPower(0);
        leftBackM.setPower(0);
        rightBackM.setPower(0);

    }

    public void timeStrafeLeft(double seconds, double inputSpeed) {

        runtime.reset();
        leftFrontM.setPower(-inputSpeed);
        rightFrontM.setPower(inputSpeed);
        leftBackM.setPower(inputSpeed);
        rightBackM.setPower(-inputSpeed);

        //Turn on the motors on so that they actually move. We forget that part a lot.
        while (opModeIsActive() && runtime.seconds() < seconds) {
            telemetry.addLine("Driving...");
        }
        leftFrontM.setPower(0);
        rightFrontM.setPower(0);
        leftBackM.setPower(0);
        rightBackM.setPower(0);
        motorMode(driveMotors, DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void encoderStrafeRight(double inches, double inputSpeed) {
        //lb & rf -1, lf & rb 1

        final double COUNTS_PER_INCH = 48.5; //COUNTS_PER_INCH is a calculation of the number of encoder ticks for every inch.
        double distance = inches;
        double speed = inputSpeed;
        leftFrontM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Reset the current encoder counts to 0;
        boolean leftStop = false;
        boolean rightStop = false;
        //Declare 2 booleans that are used to end the while() loop lower down.
        int newLeftTarget = (int) (inches * COUNTS_PER_INCH);  /*Converts the input of inches into encoder counts*/
        int newRightTarget = (int) (inches * COUNTS_PER_INCH); /**/
        leftFrontM.setTargetPosition(-newLeftTarget);
        leftBackM.setTargetPosition(newLeftTarget);
        rightFrontM.setTargetPosition(newRightTarget);
        rightBackM.setTargetPosition(-newRightTarget);

        //Set a target distance for the motors using the pre-calculated int values.
        leftFrontM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //RUN_TO_POSITION sets the motors into the mode that allows them to move using the setTargetPosition().
        //Make sure to set the motors to RUN_TO_POSITION only AFTER they have a target.
        leftFrontM.setPower(-inputSpeed);
        rightFrontM.setPower(inputSpeed);
        leftBackM.setPower(inputSpeed);
        rightBackM.setPower(-inputSpeed);

        //Turn on the motors on so that they actually move. We forget that part a lot.
        while (opModeIsActive() & !leftStop & !rightStop) { //We use opModeIsActive() in all of our while() loops.
            //This prevents the Robot Controller app from crashing when your press the stop button
            distance = inches + leftFrontM.getCurrentPosition()/COUNTS_PER_INCH;
            speed = (Math.pow(Math.abs(distance), 3) * (6.6388259 * Math.pow(10, -6)) +
                    Math.pow(Math.abs(distance), 2) * (-6.329094 * Math.pow(10, -4)) +
                    Math.abs(distance) * 0.0231008178 + .35);
            leftFrontM.setPower(-speed);
            rightFrontM.setPower(speed);
            leftBackM.setPower(speed);
            rightBackM.setPower(-speed);

            telemetry.addLine("Speed: "+(speed));
            telemetry.addLine("Left: " + leftFrontM.getCurrentPosition() + " of " + leftFrontM.getTargetPosition());
            telemetry.addLine("Right: " + rightFrontM.getCurrentPosition() + " of " + rightFrontM.getTargetPosition());
            telemetry.update();
            //Add telemetry to give us the current and target positions of both motors.
            //TODO: Create a custom telemetry for this method.
            //TODO: Change all "[telemetryName].update;" to "updateTelemetry([telemetryName]);".

            if (!leftFrontM.isBusy() || !rightFrontM.isBusy() || !leftBackM.isBusy() || rightBackM.isBusy()) { //isBusy() can only be used in RUN_TO_POSITION to check if the motor has reached their target.
                leftStop = true;
                rightStop = true;
            }
        }
        leftFrontM.setPower(0);
        rightFrontM.setPower(0);
        leftBackM.setPower(0);
        rightBackM.setPower(0);

        motorMode(driveMotors, DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //Detects the TFOD objects while strafing.
    public void encoderStrafeRightDetect(double inches, double inputSpeed) {
        //lb & rf -1, lf & rb 1

        final double COUNTS_PER_INCH = 48.5; //COUNTS_PER_INCH is a calculation of the number of encoder ticks for every inch.
        double distance = inches;
        double speed = inputSpeed;
        leftFrontM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Reset the current encoder counts to 0;
        boolean leftStop = false;
        boolean rightStop = false;
        //Declare 2 booleans that are used to end the while() loop lower down.
        int newLeftTarget = (int) (inches * COUNTS_PER_INCH);  /*Converts the input of inches into encoder counts*/
        int newRightTarget = (int) (inches * COUNTS_PER_INCH); /**/
        leftFrontM.setTargetPosition(-newLeftTarget);
        rightFrontM.setTargetPosition(newRightTarget);
        //Set a target distance for the motors using the pre-calculated int values.
        leftFrontM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //RUN_TO_POSITION sets the motors into the mode that allows them to move using the setTargetPosition().
        //Make sure to set the motors to RUN_TO_POSITION only AFTER they have a target.
        leftFrontM.setPower(inputSpeed);
        rightFrontM.setPower(inputSpeed);
        leftBackM.setPower(inputSpeed);
        rightBackM.setPower(-inputSpeed);

        //Turn on the motors on so that they actually move. We forget that part a lot.
        while (opModeIsActive() & !leftStop & !rightStop) { //We use opModeIsActive() in all of our while() loops.
            //This prevents the Robot Controller app from crashing when your press the stop button
            detectMinerals2();
            distance = inches + leftFrontM.getCurrentPosition()/COUNTS_PER_INCH;
            speed = (Math.pow(Math.abs(distance), 3) * (6.6388259 * Math.pow(10, -6)) +
                    Math.pow(Math.abs(distance), 2) * (-6.329094 * Math.pow(10, -4)) +
                    Math.abs(distance) * 0.0231008178 + .35);
            leftFrontM.setPower(-speed);
            rightFrontM.setPower(speed);
            leftBackM.setPower(0.75*speed);
            rightBackM.setPower(-0.75*speed);

            telemetry.addLine("Speed: "+(speed));
            telemetry.addLine("Left: " + leftFrontM.getCurrentPosition() + " of " + leftFrontM.getTargetPosition());
            telemetry.addLine("Right: " + rightFrontM.getCurrentPosition() + " of " + rightFrontM.getTargetPosition());
            telemetry.update();
            //Add telemetry to give us the current and target positions of both motors.
            //TODO: Create a custom telemetry for this method.
            //TODO: Change all "[telemetryName].update;" to "updateTelemetry([telemetryName]);".

            if (!leftFrontM.isBusy() || !rightFrontM.isBusy()) { //isBusy() can only be used in RUN_TO_POSITION to check if the motor has reached their target.
                leftStop = true;
                rightStop = true;
            }
        }
        leftFrontM.setPower(0);
        rightFrontM.setPower(0);
        leftBackM.setPower(0);
        rightBackM.setPower(0);

    }

    //Used internally by Vuforia
    public String formatVuf(OpenGLMatrix transformationMatrix) {
        //Honestly, we have no idea why this exists. It is called in the getVuforiaTelemetry() method.
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

    /*public String getVuMark() {
        //This is a useful method that will convert the currently visible VuMark into a String.
        vuMark = RelicRecoveryVuMark.from(relicTemplate);
        if (vuMark.equals(LEFT)) return "LEFT";
        else if (vuMark.equals(RIGHT)) return "RIGHT";
        else return "CENTER";
        //else return null;
    }*/

    // "
    public String format(OpenGLMatrix transformationMatrix) {
        //Again, we have no idea what this is for. The only difference is in the name: format() v. formatVuf().
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

    //This is one of our gyro turns from before we used a true PID.
    //It uses a logarithmic equation to determine power based on rotational distance from the target.

    /**
     * Turns using the gyro
     * @param degrees from current position (i.e. if starting at 90 and degrees = 45, moves to 135)
     * @param direction "left" or "right"
     */
    public void relativeGyroTurn(float degrees, String direction) {
        motorMode(driveMotors, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Telemetry relativeGyroTurn = new TelemetryImpl(this); //Custom method Telemetry
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX); //Update the gyro values before the turn begins
        float distance; //Distance is a somewhat misleading name, this variable actually means the amount of degrees left in the turn.
        //For the left example, the starting heading will be 135 and the degrees to turn will be 90.
        if (direction.equals("left")) {
            float target = getHeading() + degrees; //target = 135 + 90 = 225
            distance = degrees; //distance = degrees = 90
            double speed;
            if(target > 180 /*target = 225 > 180 == true*/) target -= 360; //target = 225 - 360 = -135
            idealHeading += degrees; //idealHeading is a non-specific variable used in the relativeGyroTurnIdeal() method. idealHeading = 135 + 90 = 225
            if(idealHeading > 180 /*idealHeading = 225 > 180 == true*/) idealHeading -= 360; //idealHeading = 225 - 360 = -135
            //Now, the target = -135. Using some decently simple math, we have converted the value so that it crosses the "Gap" correctly.
            runtime.reset();
            int tolerance = 3;
            while (opModeIsActive() && distance > tolerance) { //We have found that the maximum available variance in the degrees is about 8, so we turn until we are within 4 of the target.
                if(runtime.seconds() > 4) {
                    tolerance += 2;
                    runtime.reset();
                }
                /* "left", 100, start 45
                 * target = 45+100 = 145
                 * distance = 100- 45 = 55
                 * speed = (0.5) + .088 = .588
                 * speed = 0.35
                 *
                 */
                distance = target - getHeading(); //Since this is inside of the turning while() loop, distance will decrease as the robot turns. For example: --> Next line
                //1st Iteration: distance = -135 - 135 = -270
                //After we have turned 80 degrees: distance = -135 - (-145) = 10
                if (distance < 0) distance += 360;
                //1st Iteration: distance = -270 < 0 = true; distance = -270 + 360 = 90
                //After we have turned 80 degrees: distance = 10 < 0 = false; distance = 10
                //speed = 0.53028*Math.pow(1.0031, distance);
                speed = 0.08*(Math.log(distance)) + 0.05;
                //1st Iteration: speed = 0.53028*Math.pow(1.0031, 90) = 0.70
                //After we have turned 80 degrees: 0.5328*Math.pow(1.0031, 10) = 0.55
                motorTurn(speed, -speed);
                relativeGyroTurn.addLine("Current Heading: "+getHeading()+" Distance from Target: "+distance);
                System.out.println("Current Heading: "+getHeading()+" Distance from Target: "+distance+" Target: "+target);
                updateTelemetry(relativeGyroTurn);
            }
            stopDrive();
        }
        //The right turn does the exact same thing, except you will notice that it adds where left subtracts, etc. because it has to cross the gap going from negative to positive.
        else {
            float target = getHeading() - degrees;
            distance = degrees;
            double speed;
            idealHeading += degrees;
            if(idealHeading < -180) idealHeading += 360;
            if (target < -180) target += 360;
            int tolerance = 3;
            runtime.reset();
            while (opModeIsActive() && distance > tolerance) {
                if(runtime.seconds() > 4) {
                    tolerance += 2;
                    runtime.reset();
                }
                distance = getHeading() - target;
                if (distance < 0) distance += 360;
                //speed = 0.53028*Math.pow(1.0031, distance);
                speed = 0.08*(Math.log(distance)) + 0.05;
                motorTurn(-speed, speed);
                relativeGyroTurn.addLine("Current Heading: "+getHeading()+" Distance from Target: "+distance);
                updateTelemetry(relativeGyroTurn);
            }
            stopDrive();
        }
    }

    //This is our newest relativeGyroTurn method. It uses a PID (Proportional Integral Derivative) control loop to turn.

    /**
     * Turns using a PID. We discovered how difficult tuning this is, so we abandoned it.
     * The tuning process could probably be semi-automated
     * @param degrees
     * @param direction
     */
    public void gyroTurnPID(float degrees, String direction) {
        double integralSum = 0;
        double Kp, Ki, Kd;
        Kp = P;
        Ki = I;
        Kd = D;
        float priorDistance = 0;
        double priorRuntime = runtime.milliseconds();

        motorMode(driveMotors, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Telemetry relativeGyroTurn = new TelemetryImpl(this); //Custom method Telemetry
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX); //Update the gyro values before the turn begins
        float distance; //Distance is a somewhat misleading name, this variable actually means the amount of degrees left in the turn.
        //For the left example, the starting heading will be 135 and the degrees to turn will be 90.
        float target;
        double speed;
        if (direction.equals("left")) {
            target = getHeading() + degrees; //target = 135 + 90 = 225
            if(target > 180) target -= 360; //target = 225 - 360 = -135
            distance = target - getHeading(); //distance = -135 - 135 = -270
            distance += 360*is(distance < 0); //distance = -270 + 360 = 90
            distance -= 360*is(distance > 180);
            idealHeading += degrees; //idealHeading is a non-specific variable used in the relativeGyroTurnIdeal() method. idealHeading = 135 + 90 = 225
            if(idealHeading > 180) idealHeading -= 360; //idealHeading = 225 - 360 = -135
            //Now, the target = -135. Using some decently simple math, we have converted the value so that it crosses the "Gap" correctly.

        }
        //The right turn does the exact same thing, except you will notice that it adds where left subtracts, etc. because it has to cross the gap going from negative to positive.
        else {
            target = getHeading() - degrees; //target = 135 - 90 = 45
            if (target < -180) target += 360; //target = 45
            distance = target - getHeading(); //distance = 45 - 135 = -90
            distance += 360*is(distance < 0); //distance = -90 + 360 = 270
            distance -= 360*is(distance > 180); //distance = 270 - 360 = -90
            idealHeading += degrees;
            if(idealHeading < -180) idealHeading += 360;
        }
        while (opModeIsActive() && Math.abs(distance) > 1) { //We have found that the maximum available variance in the degrees is about 8, so we turn until we are within 4 of the target.
            integralSum = (0.999)*integralSum + distance;
            priorDistance = distance;
            distance = target - getHeading();
            distance += 360*is(distance < 0);
            distance -= 360*is(distance > 180);
            //1st Iteration: distance = -270 < 0 = true; distance = -270 + 360 = 90
            //After we have turned 80 degrees: distance = 10 < 0 = false; distance = 10
            //speed = 0.53028*Math.pow(1.0031, distance);
            //speed = (0.0844*Math.log(distance))*0.875)
            speed = Kp*distance + Ki*integralSum + Kd*(distance - priorDistance);
            relativeGyroTurn.addLine("dT: " + (runtime.milliseconds() - priorRuntime));
            priorRuntime = runtime.milliseconds();
            //1st Iteration: speed = 0.53028*Math.pow(1.0031, 90) = 0.70
            //After we have turned 80 degrees: 0.5328*Math.pow(1.0031, 10) = 0.55
            motorTurn(speed, -speed);
            relativeGyroTurn.addLine("Current Heading: "+getHeading()+" Distance from Target: "+distance);
            System.out.println("Current Heading: "+getHeading()+" Distance from Target: "+distance+" Target: "+target);
            relativeGyroTurn.addLine("Integral: "+integralSum+" | Derivative: "+(distance - priorDistance));
            updateTelemetry(relativeGyroTurn);
        }
        stopDrive();
    }

    /* The relativeGyroTurnIdeal() method is a product of the 5 or so meetings that we had when the competition robt was not finished, so we got bored.
     * Basically, this method calculates the target rotation using the variable idealHeading, which is calculated during each turn that incorporates the gyro.
     * So, every instance of getHeading() in the above method is replaced with idealHeading.
     */

    //@Deprecated
    public void relativeGyroTurnIdeal(float degrees, String direction) {
        Telemetry relativeGyroTurn = new TelemetryImpl(this);
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        float distance;
        if (!direction.equals("left") & !direction.equals("right")) {
            relativeGyroTurn.addLine("Direction input invalid, defaulting to left...");
            direction = "left";
            updateTelemetry(relativeGyroTurn);
            waitSec(2);
            relativeGyroTurn.clearAll();
        }
        if (direction.equals("left")) {
            float target = (idealHeading + degrees);
            distance = degrees;
            double speed;
            if(target > 180) target -= 360;
            idealHeading = target;
            while (opModeIsActive() & (distance > 4)) {
                distance = target - getHeading();
                if (distance < 0) distance += 360;
                speed = -.03173 + 0.0844*Math.log(distance);
                motorTurn(-speed, speed);
                relativeGyroTurn.addLine("Current Heading: "+getHeading()+" Distance from Target: "+distance);
                System.out.println("Current Heading: "+getHeading()+" Distance from Target: "+distance+" Target: "+target);
                updateTelemetry(relativeGyroTurn);
            }
            stopDrive();
        }
        else {
            float target = (idealHeading - degrees);
            distance = degrees;
            double speed;
            if (target < -180) target += 360;
            idealHeading = target;
            while (opModeIsActive() & (distance > 4)) {
                distance = getHeading() - target;
                if (distance < 0) distance += 360;
                speed = -.03173 + 0.0844*Math.log(distance);
                motorTurn(speed, -speed);
                relativeGyroTurn.addLine("Current Heading: "+getHeading()+" Distance from Target: "+distance);
                updateTelemetry(relativeGyroTurn);
            }
            stopDrive();
        }
    }

    public void turnToIdeal() {
        absoluteGyroTurn(idealHeading);
    }

    /**
     * Turns to the target heading
     * @param target
     */
    public void absoluteGyroTurn(float target) {
        /* This method is actually extremely simple.
         * The only difference between it and the relativeGyroTurn() method is that this one:
         * a) Converts an absolute target to a relative rotational distance
         * b) Automatically calculates whether a left or right turn is more efficient.
         */
        float distanceLeft = target - getHeading(); //Calculate the distance from the target if the robot would turn left
        float distanceRight = getHeading() - target; //Calculate the distance from the target if the robot would turn right
        if (distanceLeft < 0) distanceLeft += 360; //Clip the target to 360
        if (distanceRight < 0) distanceRight += 360; //Clip the target to 360
        if(distanceLeft <= distanceRight) { //If a left turn is more efficient than a right turn...
            relativeGyroTurn(distanceLeft, "left"); //Start a left relative gyro turn using the calculated value
        }
        else { //If a left turn is less efficient than a right turn, then do a right turn instead
            relativeGyroTurn(distanceRight, "right"); //Start a right relative gyro turn using the calculated value
        }
        idealHeading = target; //Update the idealHeading value

    }
    /* The absoluteGyroTurn() method uses the other type of turning, called absolute.
     * An absolute turn means that if the starting heading is 45 and the target input is 90, the robot will turn to the heading of 90, doing a 45 degree turn.
     */

    public void absoluteGyroTurnPID(float target) {
        /* This method is actually extremely simple.
         * The only difference between it and the relativeGyroTurn() method is that this one:
         * a) Converts an absolute target to a relative rotational distance
         * b) Automatically calculates whether a left or right turn is more efficient.
         */
        float distanceLeft = target - getHeading(); //Calculate the distance from the target if the robot would turn left
        float distanceRight = getHeading() - target; //Calculate the distance from the target if the robot would turn right
        if (distanceLeft < 0) distanceLeft += 360; //Clip the target to 360
        if (distanceRight < 0) distanceRight += 360; //Clip the target to 360
        if(distanceLeft <= distanceRight) { //If a left turn is more efficient than a right turn...
            gyroTurnPID(distanceLeft, "left"); //Start a left relative gyro turn using the calculated value
        }
        else { //If a left turn is less efficient than a right turn, then do a right turn instead
            gyroTurnPID(distanceRight, "right"); //Start a right relative gyro turn using the calculated value
        }
        idealHeading = target; //Update the idealHeading value
    }

    @Deprecated
    public void diagonalDrive(float degrees, double inches, double speed) {
        double drivePower, strafePower;
        //Declare variables that will be used to set motor power
        double COUNTS_PER_INCH = 52.3;
        //Encoder counts per inch
        leftFrontM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Reset all of the encoder counts to 0
        boolean stop = false;
        int newDriveTarget;
        int newStrafeTarget;
        //These two integers are used to set a target value when using RUN_TO_POSITION

        double ratio = Math.abs(tanDeg(degrees));
        /* Unlike our other encoder drive methods, this one needs to use trigonometry to calculate how to move diagonally.
         * The ratio value is a ratio of the speed X to the speed Y, as shown below.
         *       +Y
         *      -----
         *     |     |
         *  -X |     | +X
         *     |     |
         *      -----
         *       -Y
         */
        if(ratio > 1) {
            strafePower = 1;
            drivePower = 1/ratio;
        } else {
            drivePower = 1;
            strafePower = ratio;
        }
        //The above if-else statement adjusts either the speed X or speed Y, depending on which is lower.
        //If the speed X is higher, it is set to 1 and the speed Y is adjusted using the ratio.
        //If the speed Y is higher, it is set to 1 and the speed X is adjusted using the ratio.

        drivePower *= speed;
        strafePower *= speed;
        //Multiply both the drive and strafe powers by the maximum speed input.

        newDriveTarget = (int) (inches * cosDeg(degrees));
        //Calculates the percentage of movement that needs to be done on the Y-Axis, then multiplies it by the distance.
        newStrafeTarget = (int) (inches * sinDeg(degrees));
        //Calculates the percentage of movement that needs to be done on the X-Axis, then multiplies it by the distance.

        leftFrontM.setTargetPosition(newDriveTarget);
        rightFrontM.setTargetPosition(newDriveTarget);
        //Give the drive and strafe motors a new target to use with RUN_TO_POSITION.
        leftFrontM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontM.setPower(drivePower);
        rightFrontM.setPower(drivePower);
        //Set the motors to the adjusted powers.
        while (opModeIsActive() & !stop) {
            if (!leftFrontM.isBusy() || !rightFrontM.isBusy()) {
                leftFrontM.setPower(0);
                rightFrontM.setPower(0);
                stop = true;
                //Our standard stop loop that checks to make sure that no motor has reached its target.
            }
        }

    }

    @Deprecated
    public void timeDiagonal(float degrees, double speed, double seconds) {
        double drivePower, strafePower;
        leftFrontM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double ratio = Math.abs(tanDeg(degrees));

        if(ratio > 1) {
            strafePower = 1;
            drivePower = 1/ratio;
        } else {
            drivePower = 1;
            strafePower = ratio;
        }

        if(degrees > 0 && degrees <= 90) {
            if(ratio > 1) {
                strafePower = 1;
                drivePower = 1/ratio;
            } else {
                drivePower = 1;
                strafePower = ratio;
            }
        } else if(degrees > 90) {
            if(Math.abs(ratio) > 1) {
                strafePower = 1;
                drivePower = -1/ratio;
            } else {
                drivePower = -1;
                strafePower = ratio;
            }
        } else if(degrees < 0 && degrees >= -90) {
            if(Math.abs(ratio) > 1) {
                strafePower = -1;
                drivePower = 1/ratio;
            } else {
                drivePower = 1;
                strafePower = -ratio;
            }
        } else {
            if(Math.abs(ratio) > 1) {
                strafePower = -1;
                drivePower = -1/ratio;
            } else {
                drivePower = -1;
                strafePower = -ratio;
            }
        }
        //Does the same thing as the diagonalDrive() method's math, except this one accounts for direction

        drivePower *= speed;
        strafePower *= speed;

        Telemetry timeDiagonal = new TelemetryImpl(this);
        while (opModeIsActive() & runtime.seconds() < seconds) {
            timeDiagonal.clear();
            timeDiagonal.addLine("Moving for " + (seconds - runtime.seconds()) + " seconds");
        }
        leftFrontM.setPower(0);
        rightFrontM.setPower(0);
    }

    public int startingEncoderCount;
    public double brakeSum;
    public int priorError;

    public void setStartingEncoderCount(DcMotor motor, int position) {
        brakeSum = 0;
        startingEncoderCount = position;
        motor.setZeroPowerBehavior(BRAKE);
        priorError = 0;
    }

    public double PBrake = 0.05;
    public double IBrake = 0;
    public double DBrake = 0;
    int errorBrake = 0;


    public void liftBrake() {
        /*priorError = errorBrake;
        errorBrake = motor.getCurrentPosition() - pivotStartingEncoderCount;
        brakeSum = 0.9 * brakeSum + errorBrake;
        double speed = -(PBrake * errorBrake + IBrake * brakeSum + DBrake * (errorBrake - priorError));
        liftM.setPower(speed);
        telemetry.addLine("Start: " + pivotStartingEncoderCount);
        telemetry.addLine("Current: " + motor.getCurrentPosition());
        telemetry.addLine("Error: " + errorBrake);
        telemetry.addLine("Sum: "+brakeSum);
        telemetry.addLine("Speed: " + speed);
        telemetry.update();*/
        liftM.setPower(0.2);
    }


    public void hardBrake(DcMotor motor) {
        /*priorError = errorBrake;
        errorBrake = motor.getCurrentPosition() - pivotStartingEncoderCount;
        brakeSum = 0.9 * brakeSum + errorBrake;
        double speed = -(PBrake * errorBrake + IBrake * brakeSum + DBrake * (errorBrake - priorError));
        liftM.setPower(speed);
        telemetry.addLine("Start: " + pivotStartingEncoderCount);
        telemetry.addLine("Current: " + motor.getCurrentPosition());
        telemetry.addLine("Error: " + errorBrake);
        telemetry.addLine("Sum: "+brakeSum);
        telemetry.addLine("Speed: " + speed);
        telemetry.update();*/
        liftM.setPower(-0.3);
    }

    /**
     * For sinDeg, cosDeg, tanDeg:
     * @param degrees
     * @return the sine, cosine, or tangent of the specified degree angle
     */
    public double sinDeg(float degrees) {
        return Math.sin(Math.toRadians(degrees));
    }

    public double cosDeg(float degrees) {
        return Math.cos(Math.toRadians(degrees));
    }

    public double tanDeg(float degrees) {
        return Math.tan(Math.toRadians(degrees));
    }

    /**
     * I honestly can't remember what we ever used this for.
     * I think it was something for the mechanum drive
     * @param ratio the tangent ratio of the return angle
     * @return the angle corresponding to the input
     */
    public double atanDeg(double ratio) {
        return Math.toDegrees(Math.atan(ratio));
    }

    public void drive(double pow) {

        leftFrontM.setPower(pow);
        leftBackM.setPower(pow);
        rightFrontM.setPower(pow);
        rightBackM.setPower(pow);

    }

    ColorSensor cS;

    /**
     * Gets the current red value seen by the color sensor.
     * The color sensor will have to be properly initialized for these to work
     * @return the red value seen by the sensor
     */
    public int getRed() {
        Color.RGBToHSV(
                (int) (cS.red() * SCALE_FACTOR),
                (int) (cS.green() * SCALE_FACTOR),
                (int) (cS.blue() * SCALE_FACTOR),
                hsvValues);
        return cS.red();
    }

    /**
     * Gets the current blue value seen by the color sensor
     * @return the blue value seen by the sensor
     */
    public int getBlue() {
        Color.RGBToHSV(
                (int) (cS.red() * SCALE_FACTOR),
                (int) (cS.green() * SCALE_FACTOR),
                (int) (cS.blue() * SCALE_FACTOR),
                hsvValues);
        return cS.blue();
    }

    /**
     * One of the most useful methods ;)
     * @param bool
     * @return 1 if bool is true, 0 if bool is false
     */
    public int is(Boolean bool) {
        return (bool ? 1 : 0);
    }

    /**
     * A variation of one of the most useful methods ;)
     * @param bool
     * @return 1 if bool is true, -1 if bool is false
     */
    public int isNeg(Boolean bool) {
        return (bool ? 1 : -1);
    }

    /*
     * waitSec(seconds) is much more useful than the built-in wait() method.
     * Using the wait() method involves error-catching, which is just plain annoying.
     * waitSec(seconds) also returns telemetry that aids in timing.
     */

    /**
     * Waits the specified number of seconds, then continues
     * @param seconds the number of seconds to wait for
     */
    public void waitSec(double seconds) {
        //Create a separate timer so that we can time both the total run time and this method's.
        ElapsedTime waitSecTime = new ElapsedTime();
        waitSecTime.reset();
        while (opModeIsActive() & waitSecTime.seconds() < seconds) {
            telemetry.addLine("Waiting...");
        }
    }


    //The getHeadingTelemetry() method converts the unreadable angles.firstAngle input and converts it into degrees, then puts it on the Robot Controller.
    public void getHeadingTelemetry() {
        Telemetry headingTelemetry = new TelemetryImpl(this);
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        headingTelemetry.addData("Heading: ", AngleUnit.DEGREES.normalize(angles.firstAngle));
        updateTelemetry(headingTelemetry);
    }

    //getHeading() also normalizes the angles.firstAngle input into degrees, but returns a float instead.

    /**
     * Updates the gyro, then gets the new heading value
     * @return the current heading of the robot
     */
    public float getHeading() {
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        return AngleUnit.DEGREES.normalize(angles.firstAngle);
    }

    //stopDrive() simply stops all of the drive motors.
    public void stopDrive() {
        leftFrontM.setPower(0);
        leftBackM.setPower(0);
        rightFrontM.setPower(0);
        rightBackM.setPower(0);
    }

    public void motorMode(DcMotor[] motors, DcMotor.RunMode mode) {
        for(int i=0; i<motors.length; i++) {
            motors[i].setMode(mode);
        }
        System.out.println("Set motors to mode "+mode+".");
    }

    public void setDriveTarget(int target) {
        for(int i=0; i<driveMotors.length; i++) {
            driveMotors[i].setTargetPosition(target);
        }
    }

    //The motorRun() method is nice because it uses the strafe motor intelligently to assist with turns.
    //Notice that this method does not stop the motors, so that must be done separately.

    public double[] calcPower(int angleDeg, double speed) {
        rFPow = speed * cosDeg(angleDeg + 45);
        lFPow = speed * sinDeg(angleDeg + 45);
        rBPow = speed * sinDeg(angleDeg + 45);
        lBPow = speed * cosDeg(angleDeg + 45);


        double max = Math.max(Math.max(Math.abs(lFPow), Math.abs(lBPow)), Math.max(Math.abs(rFPow), Math.abs(rBPow)));
        if(max > 1) {
            lFPow /= max;
            lBPow /= max;
            rFPow /= max;
            rBPow /= max;
        }
        double[] array = {lFPow, rFPow, lBPow, rBPow};
        return array;
    }

    public void motorTurn(double leftSpeed, double rightSpeed) {

        leftBackM.setPower(leftSpeed);
        leftFrontM.setPower(leftSpeed);
        rightBackM.setPower(rightSpeed);
        rightFrontM.setPower(rightSpeed);

    }

    public void motorRun(int angleDeg, double speed) {

        calcPower(angleDeg, speed);

        leftBackM.setPower(powerMultiplier * lBPow);
        leftFrontM.setPower(-powerMultiplier * lFPow);
        rightBackM.setPower(powerMultiplier * rBPow);
        rightFrontM.setPower(-powerMultiplier * rFPow);

        telemetry.addLine("LF: " + round(lFPow, 1) + " | LB: " + round(lBPow, 1));
        telemetry.addLine("RF: " + round(rFPow, 1) + " | RB: " + round(rBPow, 1));

    }

    public void coastMotors() {
        leftFrontM.setZeroPowerBehavior(FLOAT);
        leftBackM.setZeroPowerBehavior(FLOAT);
        rightFrontM.setZeroPowerBehavior(FLOAT);
        rightBackM.setZeroPowerBehavior(FLOAT);
    }

    public void brakeMotors() {
        leftFrontM.setZeroPowerBehavior(BRAKE);
        leftBackM.setZeroPowerBehavior(BRAKE);
        rightFrontM.setZeroPowerBehavior(BRAKE);
        rightBackM.setZeroPowerBehavior(BRAKE);
    }

    public float round(float n, int places) {
        return (float) (Math.round(n*Math.pow(10, places)) / Math.pow(10, places));
    }
    public double round(double n, int places) {
        return (Math.round(n*Math.pow(10, places)) / Math.pow(10, places));
    }

    public void writeTelemetry(String telemetry) {
        write.clearAll();
        write.addLine(telemetry);
        updateTelemetry(write);
    }

    public void gyroStraight(double inches, double inputSpeed) {

        final float TARGET = getHeading();
        final double COUNTS_PER_INCH = 95.9; //COUNTS_PER_INCH is a calculation of the number of encoder ticks for every inch.
        int degrees = (int) (-inches*COUNTS_PER_INCH);
        boolean stop = false;
        float error = 0;
        double speedChange = 0;
        double backPowerScale = 0;
        brakeMotors();
        leftFrontM.setTargetPosition(degrees);
        rightFrontM.setTargetPosition(degrees);
        motorMode(driveMotors, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while(opModeIsActive() & !stop) {
            error = TARGET - getHeading();
            error += 360*is(error < 0);
            error -= 360*is(error > 180);
            speedChange = (double) error * kP;
            backPowerScale = 0.076*Math.pow(1.04, Math.abs(degrees - leftFrontM.getCurrentPosition()));
            leftFrontM.setPower(inputSpeed + speedChange*isNeg(inches > 0));
            leftBackM.setPower(leftFrontM.getPower()*isNeg(inches < 0)*backPowerScale);
            rightFrontM.setPower(inputSpeed - speedChange*isNeg(inches > 0));
            rightBackM.setPower(rightFrontM.getPower()*isNeg(inches < 0)*backPowerScale);

            telemetry.addLine("Speed: "+inputSpeed+" | Angle: "+getHeading());
            telemetry.addLine("Left: "+leftFrontM.getCurrentPosition()+" of "+leftFrontM.getTargetPosition());
            telemetry.addLine("Right: "+rightFrontM.getCurrentPosition()+" of "+rightFrontM.getTargetPosition());
            telemetry.addLine("Heading Error: "+error+" | Correction: "+speedChange);
            telemetry.update();

            if(!leftFrontM.isBusy() || !rightFrontM.isBusy()) {
                stop = true;
            }
        }
        stopDrive();
        waitSec(0.5);
        coastMotors();
    }

    /**
     * This is more efficient than our other ways of powering the motors,
     * because it uses an array of each motor that needs to be powered.
     * @param motors the motors to be powered
     * @param power the power that the motors should be run at
     */
    public void powerMotors(DcMotor[] motors, double power) {
        for (DcMotor motor : motors) {
            motor.setPower(power);
        }
    }

    public boolean inRange(double n1, double n2, double range) {
        return (Math.abs(n1-n2) <= range);
    }

    /*public boolean isCube() {
        Color.RGBToHSV((int) (cS.red() * SCALE_FACTOR),
                (int) (cS.green() * SCALE_FACTOR),
                (int) (cS.blue() * SCALE_FACTOR),
                hsvValues);
        return (hsvValues[1] > 0.3);
    }*/

    public int detectMinerals() {
        int moveModifier = 0;
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                if (updatedRecognitions.size() == 3) {
                    int goldMineralX = -1;
                    int silverMineral1X = -1;
                    int silverMineral2X = -1;
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldMineralX = (int) recognition.getLeft();
                        } else if (silverMineral1X == -1) {
                            silverMineral1X = (int) recognition.getLeft();
                        } else {
                            silverMineral2X = (int) recognition.getLeft();
                        }
                    }
                    if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                        if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                            telemetry.addData("Gold Mineral Position", "Left");
                            moveModifier = -1;
                        } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                            telemetry.addData("Gold Mineral Position", "Right");
                            moveModifier = 0;
                        } else {
                            telemetry.addData("Gold Mineral Position", "Center");
                            moveModifier = 1;
                        }
                    }
                }
                telemetry.update();
            }
        }
        return moveModifier;
    }


    int loc = 0;
    Recognition goldBlock;
    public void detectMinerals2() {
        int goldX = -1;
        int silver1X = -1;
        int silver2X = -1;
        updatedRecognitions = tfod.getUpdatedRecognitions();
        if(updatedRecognitions != null) {
            if (updatedRecognitions.size() == 2) {
                for (Recognition recognition : updatedRecognitions) {
                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                        goldX = (int) recognition.getBottom();
                        goldDetected++;
                        goldBlock = recognition;
                        telemetry.addLine("Gold Left: "+goldBlock.getBottom());
                        telemetry.update();
                    } else if (silver1X == -1) silver1X = (int) recognition.getBottom();
                    else silver2X = (int) recognition.getBottom();
                }
                if (silver1X != -1 && silver2X != -1) loc = 1;
                else if (goldX != -1 && silver1X != -1) {
                    if (goldX < silver1X) loc = -1;
                }
            } else {
                goldDetected = 0;
                for(int i = 0; i < updatedRecognitions.size(); i++) {
                    if(updatedRecognitions.get(i).getLabel().equals(LABEL_GOLD_MINERAL)) goldDetected++;
            }

        }

    }
}
    @Deprecated
    public void detectMinerals3() {
        int goldX = -1;
        int silver1X = -1;
        int silver2X = -1;
        updatedRecognitions = tfod.getUpdatedRecognitions();
        if(updatedRecognitions != null) {
            if (updatedRecognitions.size() == 2) {
                for (Recognition recognition : updatedRecognitions) {
                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                        goldX = (int) recognition.getBottom();
                        goldBlock = recognition;
                    } else if (silver1X == -1) silver1X = (int) recognition.getLeft();
                    else silver2X = (int) recognition.getBottom();
                }
                if (silver1X != -1 && silver2X != -1) loc = 1;
                else if (goldX != -1 && silver1X != -1) {
                    if (goldX > silver1X) loc = -1;
                }
            } else if(updatedRecognitions.size() == 1) {
                for(Recognition recognition : updatedRecognitions) {
                    if(recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                        goldX = (int) recognition.getBottom();
                        goldBlock = recognition;
                    }
                }
                if(goldX > 750) loc = 1;
                else if(goldX != -1) loc = 0;
            }
        }
    }

    Recognition[] maxes = new Recognition[2];

    double[] confidences;

    public void detectMinerals2CraterConfidence() {
        int goldX = -1;
        int silver1X = -1;
        int silver2X = -1;
        loc = 0;
        //maxes[] stores the actual recognition objects that are the closest to the phone
        maxes = new Recognition[2];
        //Update the data from the phone's camera
        //updatedRecognitions = tfod.getUpdatedRecognitions();
        updatedRecognitions = tfod.getRecognitions();
        updatedRecognitions.add(null);
        //If it actually sees something...
        if(updatedRecognitions != null) {
            //If it sees at least 2 objects (normally, it will only see 2).
            if (updatedRecognitions.size() > 1) {

                confidences = new double[updatedRecognitions.size()];
                for (int i = 0; i < updatedRecognitions.size(); i++) {
                    if(updatedRecognitions.get(i) != null) confidences[i] = updatedRecognitions.get(i).getConfidence();
                }

                maxes[0] = updatedRecognitions.get(getMax(confidences));
                updatedRecognitions.remove(getMax(confidences));
                maxes[1] = updatedRecognitions.get(getMax(confidences));

                /*updatedRecognitions.clear();
                updatedRecognitions.add(maxes[0]);
                updatedRecognitions.add(maxes[1]);
                */
                for (int i = 0; i < maxes.length; i++) {
                    if (maxes[i] != null) {
                        Recognition recognition = maxes[i];
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldX = (int) recognition.getBottom();
                            goldDetected++;
                            goldBlock = recognition;
                        } else if (silver1X == -1) silver1X = (int) recognition.getBottom();
                        else silver2X = (int) recognition.getBottom();
                    }
                }
                if (silver1X != -1 && silver2X != -1) loc = 1;
                else if (goldX != -1 && silver1X != -1) {
                    if (goldX < silver1X) loc = -1;
                }
                telemetry.addLine("Size: " + confidences.length);
                telemetry.addLine("Gold Location: " + (loc == -1 ? "LEFT" : (loc == 0 ? "CENTER" : "RIGHT")));
            }
        }
    }

    public void detectMinerals2CraterPosition() {
        int goldX = 9999999;
        int silver1X = -1;
        int silver2X = -1;
        updatedRecognitions = tfod.getUpdatedRecognitions();
        if(updatedRecognitions != null) {
            if (updatedRecognitions.size() > 1) {
                maxes = new Recognition[]{updatedRecognitions.get(0), updatedRecognitions.get(1)};
                for (int i = 0; i < updatedRecognitions.size(); i++) {
                    if (updatedRecognitions.get(i).getLeft() < maxes[0].getLeft()) {
                        maxes[0] = updatedRecognitions.get(i);
                        updatedRecognitions.remove(i);
                    }
                }

                if(maxes[0].getLeft() == maxes[1].getLeft())
                    maxes[1] = updatedRecognitions.get(0);

                for (int i = 0; i < updatedRecognitions.size(); i++) {
                    if (updatedRecognitions.get(i).getLeft() < maxes[1].getLeft()) {
                        maxes[1] = updatedRecognitions.get(i);
                        updatedRecognitions.remove(i);
                    }
                }
                updatedRecognitions.clear();
                updatedRecognitions.add(maxes[0]);
                updatedRecognitions.add(maxes[1]);
                for (Recognition recognition : updatedRecognitions) {
                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                        goldX = (int) recognition.getBottom();
                        goldDetected++;
                        goldBlock = recognition;
                        telemetry.addLine("Gold Left: " + goldBlock.getBottom());
                        telemetry.update();
                    } else if (silver1X == -1) silver1X = (int) recognition.getBottom();
                    else silver2X = (int) recognition.getBottom();
                }
                if (silver1X != -1 && silver2X != -1) loc = 1;
                else if (goldX != -1 && silver1X != -1) {
                    if (goldX < silver1X) loc = -1;
                }
            } else {
                goldDetected = 0;
                for (int i = 0; i < updatedRecognitions.size(); i++) {
                    if (updatedRecognitions.get(i).getLabel().equals(LABEL_GOLD_MINERAL))
                        goldDetected++;
                }

            }
        }
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    /**
     *
     * @param array
     * @return the index of the highest value in the array
     */
    public int getMax(double[] array) {
        int max = 0;
        for(int i = 0; i < array.length; i++) if(array[i] > array[max]) max = i;
        return max;
    }

    /**
     *
     * @param array
     * @return the index of the highest value in the array
     */
    public int getMax(int[] array) {
        int max = 0;
        for(int i = 0; i < array.length; i++) if(array[i] > array[max]) max = i;
        return max;
    }

    public void runOpMode() {

    }
}
