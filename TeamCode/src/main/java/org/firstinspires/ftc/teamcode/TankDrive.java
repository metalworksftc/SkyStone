package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TankDrive")
//@Disabled
public class TankDrive extends OpMode {
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
        rm.setPower(- gamepad1.left_stick_y);
        lm.setPower(- gamepad1.right_stick_y);

        // TODO
        //What if both triggers are pressed?

        strafeMotor.setPower(gamepad1.right_trigger - gamepad1.left_trigger);

//        Slownessing D pad controls, salt is good
//        strafeMotor.setPower(decodeDPad(gamepad1.dpad_right));
//        strafeMotor.setPower( - decodeDPad(gamepad1.dpad_left));
        lm.setPower(decodeDPad(gamepad1.dpad_up));
        rm.setPower(decodeDPad(gamepad1.dpad_up));
        lm.setPower( - decodeDPad(gamepad1.dpad_down));
        rm.setPower(decodeDPad(gamepad1.dpad_down));

        tailServo.setPosition(gamepad2.left_trigger * 0.4 + 0.6);
        stoneServo.setPosition(gamepad2.right_trigger * -0.2 + 1);

        vertical.setPower( - gamepad2.left_stick_y);
        horizontal.setPower(gamepad2.right_stick_y);

    }

    protected double decodeDPad(boolean button) {
        if (button) {
            return 0.5;
        }
        return 0;
    }
}
