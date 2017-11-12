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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

/**
 * Hyelo, frienderinos
 * This OpMode uses the common Pushbot hardware class to define the devices on the Shawn.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 * <p>
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the Shawn FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "Test: TeleOp Drive & Arm", group = "Motors")
//@Disabled
public class Shawn_TestOpMode extends OpMode {

    /* Declare OpMode members. */
    HardwareShawn Shawn = new HardwareShawn();   // Use a Shawn's hardware

    double test = 0.0;

    final int ticksPerRotation = 1440;

    @Override
    public void init() {

        Shawn.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {

        double rightPower;
        double leftPower;
        double elbowPower;
        double shoulderPower;
        double clawPower;

        if (gamepad2.dpad_right) {
            clawPower = 1;
        } else if (gamepad2.dpad_left) {
            clawPower = -1;
        } else {
            clawPower = 0;
        }

        double drive = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        shoulderPower = -(gamepad2.left_stick_y / 3);
        elbowPower = -(gamepad2.right_stick_y / 3);

        leftPower = Range.clip(drive + turn, -1.0, 1.0);
        rightPower = Range.clip(drive - turn, -1.0, 1.0);
        Shawn.leftDrive.setPower(-leftPower);
        Shawn.rightDrive.setPower(-rightPower);
        Shawn.armElbow.setPower(elbowPower);
        Shawn.armShoulder.setPower(shoulderPower);
        Shawn.armClaw.setPower(clawPower);

    }

    public void driveForward(double power) {
        Shawn.rightDrive.setPower(power);
        Shawn.leftDrive.setPower(power);
    }

    public void stopDrive() {
        driveForward(0.0);
    }

    public void turnLeft(double power) {
        Shawn.rightDrive.setPower(-power);
        Shawn.leftDrive.setPower(power);
    }

    public void turnRight(double power) {
        turnLeft(-power);
    }
}
