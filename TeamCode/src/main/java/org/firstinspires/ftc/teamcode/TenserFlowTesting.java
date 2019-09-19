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

@Autonomous(name = "TenserFlowTesting", group = "MecanumRobot")
@Disabled
public class TenserFlowTesting extends AutoMethodLib {

    public void runOpMode() {
        hardwareTFODTesting();
        telemetry.update();
        tfod.activate();
        while(!opModeIsActive() && !isStopRequested() && !isStarted()) {
            detectMinerals2CraterConfidence();
            if(confidences != null) telemetry.addLine("Size: " + confidences.length);
            if(updatedRecognitions != null) telemetry.addLine("Recog Size: " + updatedRecognitions.size());
            if(maxes != null) telemetry.addLine("Maxes size : " + maxes.length);
            if (maxes != null && updatedRecognitions != null) {
                if (maxes.length > 1 && updatedRecognitions.size() > 1) {
                        if (maxes[0].getLabel().equals(LABEL_GOLD_MINERAL) && maxes[0] != null)
                            telemetry.addLine("Found: Gold, " + maxes[0].getConfidence());
                        if (maxes[0].getLabel().equals(LABEL_SILVER_MINERAL) && maxes[0] != null)
                            telemetry.addLine("Found: Silver, " + maxes[0].getConfidence());
                        if (maxes[1].getLabel().equals(LABEL_GOLD_MINERAL) && maxes[1] != null)
                            telemetry.addLine("Found: Gold, " + maxes[1].getConfidence());
                        if (maxes[1].getLabel().equals(LABEL_SILVER_MINERAL) && maxes[1] != null)
                            telemetry.addLine("Found: Silver, " + maxes[1].getConfidence());

                }
            }
        }
    }
}
