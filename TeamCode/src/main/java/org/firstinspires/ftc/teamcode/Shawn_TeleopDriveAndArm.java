/* Copyright (c) 2017 FIRST. All rights reserved.
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

import android.media.MediaPlayer;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Test: TeleOp Drive & Arm", group = "Motors")
//@Disabled
public class Shawn_TeleopDriveAndArm extends OpMode {

    /* Declare OpMode members. */

    public static final double ARM_INCREMENT = 0.01;
    public static final double CLAW_INCREMENT = 0.03;
    public static final int LOOP_WAIT = 5;
    public static final double rightStrafeControl = 0.875;
    public static final double leftStrafeControl = 0.885;

    HardwareShawn Shawn = new HardwareShawn();   // Use a Shawn's hardware

    boolean drive = true;

    double initServoPos;

    final int ticksPerRotation = 1440;

    double armPosition = 1;
    double clawPosition;

    int numLoops = 0;

    boolean controlType = true;
    Shawn_SweepControl sweeper = null;

    // SPEECH SPEECH SPEECH SPEECH SPEECH SPEECH SPEECH SPEECH SPEECH
    MediaPlayer speech = null;
    boolean playingSpeech = false;

    @Override
    public void init() {

        Shawn.init(hardwareMap, false);


        sweeper = new Shawn_SweepControl(Shawn.hwMap);

       // speech = MediaPlayer.create(this.hardwareMap.appContext, R.raw.!!!!!!!!);  // SPEECH SPEECH SPEECH SPEECH SPEECH SPEECH
        speech = MediaPlayer.create(this.hardwareMap.appContext, R.raw.imperialmarch);
        speech.seekTo(0);
        speech.pause();

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {

        // SPEECH SPEECH SPEECH SPEECH SPEECH SPEECH SPEECH SPEECH SPEECH SPEECH SPEECH
        while (gamepad1.a) {
            if (playingSpeech) {
                playingSpeech = false;
            } else {
                playingSpeech = true;
            }
        }

        if (playingSpeech) {
            speech.start();
        } else {
            speech.pause();
        }

        if (gamepad1.dpad_up) {
            sweeper.sweepyOp(1);
        } else if (gamepad1.dpad_down) {
            sweeper.sweepyOp(-1);
        } else {
            sweeper.sweepyOp(0);
        }

        if(sweeper.White()){
            telemetry.addLine("It is white!");
        }
        else{
            telemetry.addLine("It is not white.");
        }
        telemetry.addData("Red  ", sweeper.red);
        telemetry.addData("Green", sweeper.green);
        telemetry.addData("Blue ", sweeper.blue);
        telemetry.addData("Hue", sweeper.hsvValues[0]);
        telemetry.addData("Saturation", sweeper.hsvValues[1]);
        telemetry.addData("Value", sweeper.hsvValues[2]);

    }
}
