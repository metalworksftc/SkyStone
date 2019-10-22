package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TankDrive")
//@Disabled
public class TankDrive extends OpMode {
    Servo servo, stoneServo;
    DcMotor lm, rm;


    @Override
    public void init() {
        telemetry.addLine("Running");
        lm = hardwareMap.dcMotor.get("lm");
        rm = hardwareMap.dcMotor.get("rm");
        rm.setDirection(DcMotorSimple.Direction.REVERSE);
        servo = hardwareMap.servo.get("s");
        stoneServo = hardwareMap.servo.get("ss");


    }

    @Override
    public void loop() {
        lm.setPower(gamepad1.left_stick_y);
        rm.setPower(gamepad1.right_stick_y);
        servo.setPosition(gamepad1.right_trigger * 0.4 + 0.6);
        stoneServo.setPosition(gamepad2.left_trigger);



    }
    protected void grab()  {
        stoneServo.setPosition(1);
    }
    protected void release() {
        stoneServo.setPosition(0.5);

    }






    }
