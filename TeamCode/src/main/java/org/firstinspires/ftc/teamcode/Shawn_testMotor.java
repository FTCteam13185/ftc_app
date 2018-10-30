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

@TeleOp(name = "Test: TestMotor", group = "Motors")
//@Disabled
public class Shawn_testMotor extends OpMode {

    /* Declare OpMode members. */

//    public static final double ARM_INCREMENT      = 0.01;
//    public static final double CLAW_INCREMENT     = 0.03;
//    public static final int LOOP_WAIT             = 5;
//    public static final double rightStrafeControl = 0.875;
//    public static final double leftStrafeControl  = 0.885;

    HardwareShawn Shawn = new HardwareShawn();   // Use Shawn's hardware

    boolean drive = true;

    double initServoPos;

    final int ticksPerRotation = 1440;

    double armPosition = 1;
    double clawPosition;

    int numLoops = 0;

    boolean controlType = true;

    final int GB_TICKS_PER_ROTATION = 384;
    final int SUB_ROTATION = 4;
    final int MAX_GB_TICKS = 9600;
    final int MIN_GB_TICKS = 0;
    int GBTicks = 0;

    int sweep = 0;

    double lr = 0;
    double rr = 0;
    double lf = 0;
    double rf = 0;

    @Override
    public void init() {

        Shawn.init(hardwareMap, false);

        Shawn.actuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Shawn.actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        GBTicks = Shawn.actuator.getCurrentPosition();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Initial Ticks", GBTicks);
        telemetry.update();

        Shawn.actuator.setTargetPosition(GBTicks);
        Shawn.actuator.setPower(1);

    }

    @Override
    public void loop() {

        //ACTUATOR ACTUATOR ACTUATOR ACTUATOR ACTUATOR ACTUATOR
        if (gamepad1.y) {
            GBTicks = MAX_GB_TICKS;
        }

        if (gamepad1.a) {
            GBTicks = MIN_GB_TICKS;
        }

        Shawn.actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Shawn.actuator.setTargetPosition(GBTicks);
        Shawn.actuator.setPower(1);
//
//        while(Shawn.actuator.isBusy()){}
//        Shawn.actuator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Ticks set to", GBTicks);
        telemetry.addData("Ticks Reached", Shawn.actuator.getCurrentPosition());
        telemetry.update();


        // SWEEPY THINGY SWEEPY THINGY SWEEPY THINGY SWEEPY THINGY
        if (gamepad1.x && gamepad1.b) {
            sweep = 2;
        } else if (gamepad1.x) {
            sweep = 1;
        } else if (gamepad1.b) {
            sweep = 0;
        }

        if (sweep == 2) {
            Shawn.sweepy.setPower(1);
        } else if (sweep == 1) {
            Shawn.sweepy.setPower(-1);
        } else {
            Shawn.sweepy.setPower(0);
        }


        //DRIVE DRIVE DRIVE DRIVE DRIVE DRIVE DRIVE DRIVE DRIVE DRIVE
        if (-gamepad1.left_stick_y != 0) {
            lr = -gamepad1.left_stick_y;
            lf = -gamepad1.left_stick_y;
            rr = -gamepad1.left_stick_y;
            rf = -gamepad1.left_stick_y;
        } else if (gamepad1.right_stick_y != 0) {
            lr = gamepad1.left_stick_x;
            lf = gamepad1.left_stick_x;
            rr = -gamepad1.left_stick_x;
            rf = -gamepad1.left_stick_x;
        }


    }

}
