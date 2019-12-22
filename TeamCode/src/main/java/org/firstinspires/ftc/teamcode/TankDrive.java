package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TankDrive")
//@Disabled
public class TankDrive extends OpMode {
    Servo leftTailServo,rightTailServo, stoneServo;

    DcMotor lm, rm, vertical, horizontal, strafeMotor;


    @Override
    public void init() {
        telemetry.addLine("Running");
        lm = hardwareMap.dcMotor.get("lm");
        rm = hardwareMap.dcMotor.get("rm");
        rm.setDirection(DcMotorSimple.Direction.REVERSE);
        rightTailServo = hardwareMap.servo.get("rts");
        leftTailServo = hardwareMap.servo.get("lts");
        stoneServo = hardwareMap.servo.get("ss");
        vertical =  hardwareMap.dcMotor.get("tm");
        horizontal = hardwareMap.dcMotor.get("bm * 0.5");
        strafeMotor = hardwareMap.dcMotor.get("sm");
    }

    @Override
    public void loop() {
//        Gamepad1

        rm.setPower(- gamepad1.left_stick_y);
        lm.setPower(- gamepad1.right_stick_y);

        strafeMotor.setPower(gamepad1.right_trigger - gamepad1.left_trigger);

        rightTailServo.setPosition(decodeBumper(gamepad1.left_bumper));
        leftTailServo.setPosition(decodeBumper(gamepad1.left_bumper) * -1 + 1);

//        Gamepad2

        vertical.setPower( - gamepad2.left_stick_y);
        horizontal.setPower(gamepad2.right_stick_y);

        stoneServo.setPosition(gamepad2.right_trigger * -0.2 + 1);

    }

    protected double decodeBumper(boolean button) {
        if (button) {
            return 1;
        }
        return 0;
    }
}
