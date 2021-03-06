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

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

/*
 *
 * This is an example LinearOpMode that shows how to use
 * a Modern Robotics Color Sensor.
 *
 * The op mode assumes that the color cSensor
 * is configured with a name of "sensor_color".
 *
 * You can use the X button on gamepad1 to toggle the LED on and off.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name = "Sensor: REV Color", group = "Sensor")
@Disabled
public class Shawn_SensorMRColor extends LinearOpMode {

  ColorSensor colorSensor;    // Hardware Device Object
  DistanceSensor sensorDistance;


  @Override
  public void runOpMode() {

    // Variables to hold the color values
    int RED;
    int GREEN;
    int BLUE;

    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValues[] = {0F,0F,0F};

    // values is a reference to the hsvValues array.
    final float values[] = hsvValues;

    // sometimes it helps to multiply the raw RGB values with a scale factor
    // to amplify/attentuate the measured values.
    final int SCALE_FACTOR = 1;

    // get a reference to the RelativeLayout so we can change the background
    // color of the Robot Controller app to match the hue detected by the RGB cSensor.
    int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
    final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

    // get a reference to our ColorSensor object.
    colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");

    // get a reference to the distance cSensor that shares the same name.
    sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");

    // Set the LED in the beginning
    colorSensor.enableLed(false);

    // wait for the start button to be pressed.
    waitForStart();

    // while the op mode is active, loop and read the RGB data.
    // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
    while (opModeIsActive()) {

      // convert the RGB values to HSV values.
      RED   = colorSensor.red() * SCALE_FACTOR;
      GREEN = colorSensor.green() * SCALE_FACTOR;
      BLUE  = colorSensor.blue() * SCALE_FACTOR;
      Color.RGBToHSV(RED, GREEN, BLUE, hsvValues);

      // send the info back to driver station using telemetry function.
      telemetry.addData("Distance (cm)",
              String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
      //telemetry.addData("Clear", colorSensor.alpha());
      telemetry.addData("Red  ", RED);
      telemetry.addData("Green", GREEN);
      telemetry.addData("Blue ", BLUE);
      telemetry.addData("Hue", hsvValues[0]);
      if((RED>BLUE) && (RED>GREEN)){
        telemetry.addLine("It's red");
      }
      if((BLUE>RED) && (BLUE>GREEN)) {
        telemetry.addLine("It's blue");
      }

      // change the background color to match the color detected by the RGB cSensor.
      // pass a reference to the hue, saturation, and value array as an argument
      // to the HSVToColor method.
      relativeLayout.post(new Runnable() {
        public void run() {
          relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
        }
      });

      telemetry.update();
    }

    // Set the panel back to the default color
    relativeLayout.post(new Runnable() {
      public void run() {
        relativeLayout.setBackgroundColor(Color.WHITE);
      }
    });

  }

  // thing

  public boolean isBlue(ColorSensor sensor) {
      int RED = sensor.red() * 1;
      int BLUE = sensor.blue() * 1;
      int GREEN = sensor.green() * 1;
      if (BLUE > RED && BLUE > GREEN){
          return true;
      } else {
          return false;
      }
  }

}
