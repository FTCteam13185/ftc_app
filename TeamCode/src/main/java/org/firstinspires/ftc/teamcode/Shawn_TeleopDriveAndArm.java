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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

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
    Shawn_ColorSorter colorSorter = null;

    @Override
    public void init() {

        Shawn.init(hardwareMap, false);


        colorSorter = new Shawn_ColorSorter(Shawn.hwMap);



        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {

        if (colorSorter.White()) {
            Shawn.leftRear.setPower(0);

            telemetry.addLine("It is White");

        }
        else{
                if (gamepad1.dpad_up) {
                    Shawn.leftRear.setPower(1);
                } else if (gamepad1.dpad_down) {
                    Shawn.leftRear.setPower(-1);
                } else {
                    Shawn.leftRear.setPower(0);
                }

                telemetry.addLine("It is not White");
        }
        telemetry.addData("Red  ", colorSorter.red);
        telemetry.addData("Green", colorSorter.green);
        telemetry.addData("Blue ", colorSorter.blue);
        telemetry.addData("Hue", colorSorter.hsvValues[0]);
        telemetry.addData("Saturation", colorSorter.hsvValues[1]);
        telemetry.addData("Value", colorSorter.hsvValues[2]);

    }
}
