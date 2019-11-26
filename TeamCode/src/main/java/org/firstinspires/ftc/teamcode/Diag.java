package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Diag")
//@Disabled
public class Diag extends OpMode {
    Servo tailServo, stoneServo;

    DcMotor lm, rm, vertical, horizontal, strafeMotor;


    @Override
    public void init() {
        telemetry.addLine("Running");
        lm = hardwareMap.dcMotor.get("lm");
        rm = hardwareMap.dcMotor.get("rm");
        rm.setDirection(DcMotorSimple.Direction.REVERSE);
        tailServo = hardwareMap.servo.get("ts");
        stoneServo = hardwareMap.servo.get("ss");
        vertical =  hardwareMap.dcMotor.get("tm");
        horizontal = hardwareMap.dcMotor.get("bm");
        strafeMotor = hardwareMap.dcMotor.get("sm");
    }

    @Override
    public void loop() {
        telemetry.addLine("left trigger: " + gamepad1.left_trigger + " right trigger " + gamepad1.right_trigger);
        telemetry.update();
    }

    protected double decodeDPad(boolean button) {
        if (button) {
            return 0.5;
        }
        return 0;
    }
}
