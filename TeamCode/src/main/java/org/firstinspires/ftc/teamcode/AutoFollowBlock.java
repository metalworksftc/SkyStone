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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

@Autonomous(name = "AutoFollowBlock", group = "MecanumRobot")
@Disabled
public class AutoFollowBlock extends AutoMethodLib {

    double angle = 0;
    double power = 0.4;
    List<Recognition> updatedRecognitions = null;
    Servo fishingServo;
    double fishPosition = 0.5;
    int fishDirection = 1;
    public void runOpMode() {
        //L: -57
        //C: -90
        //R: -125.625
        hardwareMW();
        fishingServo = hardwareMap.servo.get("fs");
        tfod.activate();
        while(!isStarted()) {
            telemetry.addLine("Initialized");
            updatedRecognitions = tfod.getUpdatedRecognitions();
            if(updatedRecognitions != null) telemetry.addLine("Objects Detected: "+updatedRecognitions.size());
            telemetry.update();
        }
        while(opModeIsActive()) {
            fishPosition += 0.0001 * fishDirection;
            if(fishPosition >= 0.65 || fishPosition <= 0.3) fishDirection = -fishDirection;
            fishingServo.setPosition(fishPosition);
            telemetry.addLine("Angle: "+angle);
            telemetry.addLine("Position: "+fishPosition+" | Direction: "+fishDirection);
            telemetry.update();
            Recognition closest;
            try {
                updatedRecognitions = tfod.getUpdatedRecognitions();

                if (updatedRecognitions.size() == 1 && updatedRecognitions.get(0).getLabel().equals(LABEL_GOLD_MINERAL)) {
                    angle = updatedRecognitions.get(0).estimateAngleToObject(AngleUnit.DEGREES);
                } else if(updatedRecognitions.size() > 1) {
                    closest = updatedRecognitions.get(0);
                    for(Recognition recognition : updatedRecognitions) {
                        if(recognition.getWidth() > closest.getWidth()) closest = recognition;
                    }
                    angle = closest.estimateAngleToObject(AngleUnit.DEGREES);
                } else angle = 0;
                power = (0.085 * Math.log(Math.abs(angle))) * 0.875;
                if (angle > 0) motorTurn(power, -power);
                else if (angle < 0) motorTurn(-power, power);
            } catch(Exception e) {
                telemetry.addLine("Error: "+e.getLocalizedMessage());
            }
        }
    }
}
