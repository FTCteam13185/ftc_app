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

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import android.media.MediaPlayer;

import java.io.File;


@TeleOp(name = "Test: TestSound", group = "Sounds")
//@Disabled
public class Shawn_testSound extends OpMode {


//    SoundPlayer sPlayer = new SoundPlayer(1,4*1024*1024);
//    File lightsaber = new File("/sdcard/Music/LightsaberTurnOn.mp3");
//    SoundPlayer.PlaySoundParams playerParam = new SoundPlayer.PlaySoundParams();
    MediaPlayer imperialMarch = null;
    MediaPlayer lightsaber = null;

    int control = 1;
    long i = 1;

    @Override
    public void init() {

//        if (lightsaber.exists()) {
//            telemetry.addLine(lightsaber.getAbsolutePath());
//            telemetry.addData("Master volume is set to: ", sPlayer.getMasterVolume());
//            String fileList[] = musicFile.list();
//            int n = fileList.length;
//            if (n>0) {
//                telemetry.addData("Number of files: ", n);
//                for (int j = 0; j < n; j++) {
//                    telemetry.addLine(fileList[j]);
//                }
//            }
//            telemetry.update();
//            playerParam.loopControl = 0;
//            playerParam.rate = 1;
//            playerParam.volume = 2;
//            playerParam.waitForNonLoopingSoundsToFinish = true;
//        }

        imperialMarch = MediaPlayer.create(this.hardwareMap.appContext,R.raw.imperialmarch);
        imperialMarch.seekTo(0);
        lightsaber = MediaPlayer.create(this.hardwareMap.appContext,R.raw.lightsaber);
        lightsaber.seekTo(0);
//        if (musicFile.exists() && false) {
//        }
//        else {
//            telemetry.addLine("Cannot open File!!!");
//            if (!musicFile.isFile()) {
//                telemetry.addLine("This is not a File!");
//            }
//            fileList = musicFile.list();
//            telemetry.addData("First file is ", fileList[0]);
//            telemetry.addData("Second file is ", fileList[0]);
//            telemetry.addData("Third file is ", fileList[0]);
//        }
//        telemetry.addData("Status", "Initialized");

    }

    @Override
    public void loop() {
        if (gamepad1.x && !lightsaber.isPlaying()) {
//            sPlayer.preload(this.hardwareMap.appContext, lightsaber);
//            sPlayer.play(this.hardwareMap.appContext, lightsaber, 2,0,1);
            lightsaber.seekTo(0);
            lightsaber.start();
        }

        if (imperialMarch.isPlaying()) {
            telemetry.addLine("Imperial March is playing!!!");
            telemetry.addData("Current position: ", imperialMarch.getAudioSessionId());
            telemetry.update();
            if (gamepad1.a || (imperialMarch.getCurrentPosition() >= 50000)) {
//            sPlayer.preload(this.hardwareMap.appContext, imperialMarch);
//            sPlayer.play(this.hardwareMap.appContext, imperialMarch, 2,0,1);
                imperialMarch.pause();
                imperialMarch.seekTo(0);
            }
        }
        else {
            telemetry.addLine("Imperial March is not playing");
            if (gamepad1.y) {
//            sPlayer.preload(this.hardwareMap.appContext, imperialMarch);
//            sPlayer.play(this.hardwareMap.appContext, imperialMarch, 2,0,1);
                telemetry.addLine("Y pressed");
//                imperialMarch.seekTo(0);
                imperialMarch.start();
            }
            telemetry.update();
        }
    }

}
