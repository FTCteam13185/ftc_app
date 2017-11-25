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
public class Shawn_TestOpMode extends OpMode {

    /* Declare OpMode members. */

    public static final int TICKS_PER_DEGREE    = 4;
    public static final int MAX_ARM_INCREMENT   = TICKS_PER_DEGREE * 5;
    public static final double ARM_POWER        = 1;

    HardwareShawn Shawn = new HardwareShawn();   // Use a Shawn's hardware

    double test = 0.0;

    boolean drive = true;

    int currentShoulderPosition = 0;
    int currentElbowPosition = 0;

    int initArmElbow;
    int initArmShoulder;

    final int ticksPerRotation = 1440;

    @Override
    public void init() {

        Shawn.init(hardwareMap);

        initArmElbow = Shawn.armElbow.getCurrentPosition();
        initArmShoulder = Shawn.armShoulder.getCurrentPosition();


        telemetry.addData("Status", "Initialized");

    }

    @Override
    public void loop() {

        double rightPower;
        double leftPower;
        double clawPower;
        double leftStick;
        double rightStick;

        if (gamepad2.dpad_right) {
            clawPower = 1;
        } else if (gamepad2.dpad_left) {
            clawPower = -1;
        } else {
            clawPower = 0;
        }

        double drive = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        leftPower = Range.clip(drive + turn, -1.0, 1.0);
        rightPower = Range.clip(drive - turn, -1.0, 1.0);
        Shawn.leftDrive.setPower(leftPower);
        Shawn.rightDrive.setPower(rightPower);
        Shawn.armClaw.setPower(clawPower);

        leftStick = -gamepad2.left_stick_y;
        rightStick = -gamepad2.right_stick_y;
        currentShoulderPosition = moveArm(Shawn.armShoulder, leftStick, currentShoulderPosition, initArmShoulder);
        currentElbowPosition = moveArm(Shawn.armElbow, rightStick, currentElbowPosition, initArmElbow);

    }

    public int moveArm(DcMotor armMotor, double stick, int currentPosition, int initPosition) {

        if (stick > 0){
            currentPosition = armMotor.getCurrentPosition();
            armMotor.setTargetPosition(currentPosition + (int)(MAX_ARM_INCREMENT * stick));
            armMotor.setPower(ARM_POWER);
        } else if (stick < 0 && currentPosition > initPosition) {
            currentPosition = armMotor.getCurrentPosition();
            armMotor.setTargetPosition(currentPosition + (int)(MAX_ARM_INCREMENT * stick));
            armMotor.setPower(ARM_POWER);
        } else {
            armMotor.setTargetPosition(currentPosition);
        }
        return currentPosition;
    }

}
