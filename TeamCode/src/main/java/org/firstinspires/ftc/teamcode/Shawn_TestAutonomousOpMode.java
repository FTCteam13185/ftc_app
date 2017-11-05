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

import android.content.Context;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

@Autonomous(name = "Test: Autonomous Something", group = "Pushbot")
@Disabled
public class Shawn_TestAutonomousOpMode extends LinearOpMode {

//    experimentation
//    HardwarePushbot Shawn = new HardwarePushbot();
//    HardwareMap thing = new HardwareMap(hardwareMap.appContext);

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = -2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    DcMotor rightDrive = null;
    DcMotor leftDrive = null;

    double test = 0.0;

    double robotDiameter;

    Shawn_TestAutonomousOpMode() {
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");

        rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robotDiameter = 14.25;

    }


//    autonomous has no init
//    @Override
//    public void init() {
//
//        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
//        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
//
//        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        rightDrive.setDirection(DcMotor.Direction.FORWARD);
//        leftDrive.setDirection(DcMotor.Direction.REVERSE);
//
//        telemetry.addData("Status", "Initialized");
//   }

    @Override
    public void runOpMode(){

       // Shawn.init(hardwareMap); //idk what this does

        Shawn_TestAutonomousOpMode Sheep = new Shawn_TestAutonomousOpMode();

        driveForwardDistance(1, (int)COUNTS_PER_INCH * 12);

        turnRight(90, 1);

    }

    public void driveForward(double power) {
        rightDrive.setPower(power);
        leftDrive.setPower(power);
    }

    public void stopDrive() {
        driveForward(0.0);
    }

    public void turnRight(int degrees, int power) {
        double distance = (degrees/360) * (robotDiameter * 3.141592653589793238462643383279);

        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightDrive.setTargetPosition((int)distance);
        leftDrive.setTargetPosition((int)distance);

        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightDrive.setPower(power);
        leftDrive.setPower(power);

        while(rightDrive.isBusy() && leftDrive.isBusy()) {

        }

        stopDrive();
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void turnLeft(int degrees, int power) {
        turnRight(-degrees, power);
    }

    public void driveForwardDistance(double power, int distance) {
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightDrive.setTargetPosition(distance);
        leftDrive.setTargetPosition(distance);

        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        driveForward(power);

        while(rightDrive.isBusy() && leftDrive.isBusy()) {

        }

        stopDrive();
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
