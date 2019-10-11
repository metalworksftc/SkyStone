package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TankDrive")
//@Disabled
public class TankDrive extends OpMode {
    Servo servo;
    DcMotor lm, rm;


    @Override
    public void init() {
        telemetry.addLine("Running");
        lm = hardwareMap.dcMotor.get("lm");
        rm = hardwareMap.dcMotor.get("rm");
        rm.setDirection(DcMotorSimple.Direction.REVERSE);
        servo = hardwareMap.servo.get("s");


    }

    @Override
    public void loop() {
        lm.setPower(gamepad1.left_stick_y *0.5);
        rm.setPower(gamepad1.right_stick_y * 0.5);
        servo.setPosition(gamepad1.right_trigger);


    }







}
