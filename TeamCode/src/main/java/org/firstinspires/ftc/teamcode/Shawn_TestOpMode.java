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
    public static final int MAX_ARM_INCREMENT   = TICKS_PER_DEGREE * 8;
    public static final double ARM_POWER        = 1;

    HardwareShawn Shawn = new HardwareShawn();   // Use a Shawn's hardware

    double test = 0.0;

    boolean drive = true;

    int currentShoulderPosition = 0;
//
//    int stay;

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
        int currentElbowPosition;

        if (gamepad2.dpad_right) {
            clawPower = 1;
        } else if (gamepad2.dpad_left) {
            clawPower = -1;
        } else {
            clawPower = 0;
        }

        double drive = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
//        shoulderPower = Range.clip(-(gamepad2.left_stick_y), -0.5, 0.5);
//        elbowPower = Range.clip(-(gamepad2.right_stick_y), -0.5, 0.5);
//        shoulderPower = -(gamepad2.left_stick_y);
//        elbowPower = -(gamepad2.right_stick_y);

        leftPower = Range.clip(drive + turn, -1.0, 1.0);
        rightPower = Range.clip(drive - turn, -1.0, 1.0);
        Shawn.leftDrive.setPower(leftPower);
        Shawn.rightDrive.setPower(rightPower);
        Shawn.armClaw.setPower(clawPower);
 //       Shawn.armElbow.setPower(elbowPower);

   //     currentElbowPosition = Shawn.armElbow.getCurrentPosition();

//        if (-gamepad2.right_stick_y > 0){
//           Shawn.armElbow.setTargetPosition(currentElbowPosition + 1);
//        } else if (-gamepad2.right_stick_y < 0){
//            Shawn.armElbow.setTargetPosition(currentElbowPosition - 1);
//        }

        double leftStick = -gamepad2.left_stick_y;

        if (leftStick > 0) {
            currentShoulderPosition = Shawn.armShoulder.getCurrentPosition();
            Shawn.armShoulder.setTargetPosition(currentShoulderPosition + (int)(MAX_ARM_INCREMENT * leftStick));
            Shawn.armShoulder.setPower(ARM_POWER);
            telemetry.addLine("up");
            telemetry.addData("Current Position: ", "%4d", currentShoulderPosition);
            telemetry.addData("Power: ", "%3.1f", Shawn.armShoulder.getPower());
        } else if (leftStick < 0){
            currentShoulderPosition = Shawn.armShoulder.getCurrentPosition();
            Shawn.armShoulder.setTargetPosition(currentShoulderPosition + (int)(MAX_ARM_INCREMENT * -1));
            Shawn.armShoulder.setPower(ARM_POWER);
            telemetry.addLine("down");
            telemetry.addData("Current Position: ", "%4d", currentShoulderPosition);
            telemetry.addData("Power: ", "%3.1f", Shawn.armShoulder.getPower());
        } else {
            Shawn.armShoulder.setTargetPosition(currentShoulderPosition);
            telemetry.addLine("stay");
        }
        telemetry.update();

    //    ArmPosition(Shawn.armShoulder, -gamepad2.left_stick_y);
    //    ArmPosition(Shawn.armElbow, 0.5);

}

    public void ArmPosition(DcMotor armMotor, double power){

        int stayPosition = 0;

        if (power != 0) {
            if (this.drive) {
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                this.drive = false;
            }
//            armMotor.setTargetPosition(armMotor.getCurrentPosition() + 1);
        } else {
            if (!this.drive) {
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                this.drive = true;
                stayPosition = armMotor.getCurrentPosition();
                armMotor.setPower(ARM_POWER);
            }
            armMotor.setTargetPosition(stayPosition);
        }
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
