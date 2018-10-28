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
import com.qualcomm.hardware.ams.AMSColorSensor;
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
//@Disabled
public class Shawn_NewSensorColor extends LinearOpMode {

  AMSColorSensor colorSensor;    // Hardware Device Object
//  DistanceSensor sensorDistance;


  @Override
  public void runOpMode() {

    // Variables to hold the color values
    int red;
    int green;
    int blue;
    int alpha;

    float prcntRed;
    float prcntGreen;
    float prcntBlue;

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
    colorSensor = hardwareMap.get(AMSColorSensor.class, "sensor_color");

    // get a reference to the distance cSensor that shares the same name.
//    sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");

    // Set the LED in the beginning
    colorSensor.enableLed(true);

    // wait for the start button to be pressed.
    waitForStart();

    // while the op mode is active, loop and read the RGB data.
    // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
    while (opModeIsActive()) {

      // convert the RGB values to HSV values.
      red   = colorSensor.red() * SCALE_FACTOR;
      green = colorSensor.green() * SCALE_FACTOR;
      blue  = colorSensor.blue() * SCALE_FACTOR;
      alpha = colorSensor.alpha() * SCALE_FACTOR;
      Color.RGBToHSV(red, green, blue, hsvValues);

      prcntRed   = (float)red/(float)alpha;
      prcntGreen = (float)green/(float)alpha;
      prcntBlue  = (float)blue/(float)alpha;

      // send the info back to driver station using telemetry function.
//      telemetry.addData("Distance (cm)",
//              String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
      //telemetry.addData("Clear", colorSensor.alpha());
      telemetry.addData("Red  ",  red);
      telemetry.addData("Green",  green);
      telemetry.addData("Blue ",  blue);
      telemetry.addData("Alpha ", alpha);
      telemetry.addData("Alpha 2", red+green+blue);
      telemetry.addData("% Red  ", prcntRed);
      telemetry.addData("% Green", prcntGreen);
      telemetry.addData("% Blue ", prcntBlue);

      if((prcntRed>0.9/3) && (prcntGreen>0.9/3) && (prcntBlue>0.9/3)){
        telemetry.addLine("It's white");
      }

      // change the background color to match the color detected by the RGB cSensor.
      // pass a reference to the hue, saturation, and value array as an argument
      // to the HSVToColor method.
//      relativeLayout.post(new Runnable() {
//        public void run() {
//          relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
//        }
//      });

      telemetry.update();
    }

    // Set the panel back to the default color
//    relativeLayout.post(new Runnable() {
//      public void run() {
//        relativeLayout.setBackgroundColor(Color.WHITE);
//      }
//    });

  }

  // thing
}
