/* Copyright (c) 2018 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * This 2018-2019 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the gold and silver minerals.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */

public class Shawn_TensorDetectionCLASS {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = "AdC2UuL/////AAAAmbSzzw4/ykWZk7KXU2Ee5ktIYR7RAtJsPrHto/zr/+Lbg1yivLyOllic76kSLHyg2pVgyK+O1gc28/qTWiKCP8WOCzNZ6cq1WMeHspqwVy2jAEN2uR/L/knOn6MO2mqToCJX4zwu15GGIlEyAdbkYKC996Rl3vWD1gtojsWjbAsiVeWVTcfRpENlJA4B/jKsoQHnrzvHIbBV+K5cFh2nYU12jwN8UyM0gUdPPGvspDPVeti8gTKXl+RGddwkIgoLJD0W+Qy0VlCq0j/85C1b72E2yAbFYsIs5GSuOtuYJZw09a+sssGGnbUEXBeUT2mPi607EIU4ga0gcI4gLEo4Ry/qxLBX6v056cXpZrx2JOQW";;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    LinearOpMode opmode;
    HardwareMap hardwareMap;
    int screenLimit;

    public Shawn_TensorDetectionCLASS(LinearOpMode om, HardwareMap hwMap, int screenLimit) {
        opmode = om;
        hardwareMap = hwMap;
        this.screenLimit = screenLimit;
    }

    public void init() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.FRONT;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.

        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL);
    }

    public boolean detectGold(double runTime) {
        double startTime = opmode.getRuntime();
        boolean seen = false;
        if (tfod != null) {
            tfod.activate();
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            while (opmode.opModeIsActive() && opmode.getRuntime()-startTime < runTime && !seen) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    opmode.telemetry.addData("# Object Detected", updatedRecognitions.size());
                    if (updatedRecognitions.size() > 0) {
                        opmode.telemetry.addData("image height", updatedRecognitions.get(0).getImageHeight());
                        opmode.telemetry.addData("image width", updatedRecognitions.get(0).getImageWidth());
                    }
                    if (updatedRecognitions.size() >= 1) {
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getBottom() > screenLimit) {
                                opmode.telemetry.addData("bottom position", recognition.getBottom());
                                opmode.telemetry.addLine("GOLD!");
                                seen = true;
                            }
                        }
                    } else {
                        seen = false;
                    }
                    opmode.telemetry.update();
                }
            }
        }
        return seen;
    }
}
