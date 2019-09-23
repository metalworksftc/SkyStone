package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "TankDrive")
//@Disabled
public class TankDrive extends OpMode {

    DcMotor lm, rm;


    @Override
    public void init() {
        telemetry.addLine("Running");
        lm = hardwareMap.dcMotor.get("lm");
        rm = hardwareMap.dcMotor.get("rm");
        lm.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void loop() {
        lm.setPower(gamepad1.left_stick_y);
        rm.setPower(gamepad1.right_stick_y);



    }







}
