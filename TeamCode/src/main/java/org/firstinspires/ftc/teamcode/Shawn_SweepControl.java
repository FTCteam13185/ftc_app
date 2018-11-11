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

import android.graphics.Color;

import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
public class Shawn_SweepControl {

  AMSColorSensor colorSensor;    // Hardware Device Object

  public DcMotor sweepy = null;


    // Variables to hold the color values
  float red;
  float green;
  float blue;
  float alpha;
  float percentG;
  float percentB;
  float percentR;


  // sometimes it helps to multiply the raw RGB values with a scale factor
  // to amplify/attentuate the measured values.
  final int SCALE_FACTOR = 1;

  float hsvValues[] = {0F,0F,0F};

    // values is a reference to the hsvValues array.
    final float values[] = hsvValues;

    // get a reference to the RelativeLayout so we can change the background
  // color of the Robot Controller app to match the hue detected by the RGB cSensor.
 // int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
  //final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

  public Shawn_SweepControl(HardwareMap hwmap ) {

    // get a reference to our ColorSensor object.
    colorSensor = hwmap.get(AMSColorSensor.class, "colorSensor");

    // Turning on color sensor light
    colorSensor.enableLed(true);

    sweepy = hwmap.get(DcMotor.class, "sweepy");

    sweepy.setDirection(DcMotorSimple.Direction.REVERSE);
    sweepy.setPower(0);
    sweepy.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    sweepy.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


  }

  public boolean White () {


      // convert the RGB values to HSV values.


      red   = colorSensor.red() * SCALE_FACTOR;
      green = colorSensor.green() * SCALE_FACTOR;
      blue  = colorSensor.blue() * SCALE_FACTOR;

      alpha = colorSensor.alpha() * SCALE_FACTOR;

      percentR = red/alpha;
      percentB = blue/alpha;
      percentG = green/alpha;

      Color.RGBToHSV((int)red, (int)green, (int)blue, hsvValues);

      // Sending Driver station true or false based on the color white
      //if (percentR > 0.85/3 && percentB > 0.85/3 && percentG > 0.85/3) {
      if ( hsvValues[0] > 115 && hsvValues[0] < 175 ) {
          return true;
      }
      else return false;
    }

    public void sweepyOp (int pow) {
        if (this.White()) {
            sweepy.setPower(0);
        }
        else{
            sweepy.setPower(pow);
        }

    }

  }




