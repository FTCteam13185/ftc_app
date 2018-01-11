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
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Test: TeleOp Drive & Arm", group = "Motors")
//@Disabled
public class Shawn_TeleopDriveAndArm extends OpMode {

    /* Declare OpMode members. */

    public static final int TICKS_PER_DEGREE    = 4;
    public static final int MAX_ARM_INCREMENT   = TICKS_PER_DEGREE * 5;
    public static final double ARM_POWER        = 1;
    public static final double ARM_INCREMENT    = 0.01;
    public static final int LOOP_WAIT           = 10;

    HardwareShawn Shawn = new HardwareShawn();   // Use a Shawn's hardware

    boolean drive = true;

    int initArmElbow;
    int initArmShoulder;

    double initServoPos;

    final int ticksPerRotation = 1440;

    double armPosition;
    double clawPosition;

    int numLoops = 0;

    @Override
    public void init() {

        Shawn.init(hardwareMap, false);

        initArmElbow = Shawn.armElbow.getCurrentPosition();
        initArmShoulder = Shawn.armShoulder.getCurrentPosition();
//
//        Shawn.armServo.setPosition(0);
        initServoPos = Shawn.armServo.getPosition();

        Shawn.rightClaw.setPosition(clawPosition);
        Shawn.leftClaw.setPosition(1 - clawPosition);

        telemetry.addData("Initial servo armPosition", "%4.2f", initServoPos);
        telemetry.addData("Status", "Initialized");

    }

    @Override
    public void loop() {

        double rightPower;
        double leftPower;
        double clawPower;
        double leftStick;
        double rightStick;

        double drive = -gamepad1.left_stick_y;
        double turn = (-gamepad1.right_stick_x)/2;

        leftPower = Range.clip(drive + turn, -1.0, 1.0);
        rightPower = Range.clip(drive - turn, -1.0, 1.0);
        Shawn.leftDrive.setPower(leftPower);
        Shawn.rightDrive.setPower(rightPower);

        // code for mecanum wheels:
//        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
//        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
//        double rightX = gamepad1.right_stick_x;
//        final double v1 = r * Math.cos(robotAngle) + rightX;
//        final double v2 = r * Math.sin(robotAngle) - rightX;
//        final double v3 = r * Math.sin(robotAngle) + rightX;
//        final double v4 = r * Math.cos(robotAngle) - rightX;
//
//        Shawn.leftFront.setPower(v1);
//        Shawn.rightFront.setPower(v2);
//        Shawn.leftRear.setPower(v3);
//        Shawn.rightRear.setPower(v4);

        if (numLoops == LOOP_WAIT) {
            if (-gamepad2.left_stick_y < 0) {
                if (armPosition < 1) {
                    armPosition += ARM_INCREMENT;
                    if (armPosition > 1) {
                        armPosition = 1;
                    }
                }
            } else if (-gamepad2.left_stick_y > 0) {
                if (armPosition > 0) {
                    armPosition -= ARM_INCREMENT;
                    if (armPosition < 0) {
                        armPosition = 0;
                    }
                }
            }
        }

        Shawn.armServo.setPosition(armPosition);

        telemetry.addData("gamepad2.left_stick_y: ", "%4.2f", -gamepad2.left_stick_y);
        telemetry.addData("servo armPosition: ", "%4.2f", Shawn.armServo.getPosition());
        telemetry.addData("armPosition: ", "%4.2f", armPosition);

        if (numLoops == LOOP_WAIT) {
            if (gamepad2.right_stick_x > 0) {
                if (clawPosition < 1) {
                    clawPosition += ARM_INCREMENT*3;
                    if (clawPosition > 1) {
                        clawPosition = 1;
                    }
                }
            } else if (gamepad2.right_stick_x < 0) {
                if (clawPosition > 0) {
                    clawPosition -= ARM_INCREMENT*3;
                    if (clawPosition < 0) {
                        clawPosition = 0;
                    }
                }
            }
            numLoops = 0;
        }

        if (gamepad2.x) {
            clawPosition = 0;
        }

        Shawn.rightClaw.setPosition(clawPosition - 0.05);
        Shawn.leftClaw.setPosition(1 - clawPosition);

        numLoops += 1;

    }

}
