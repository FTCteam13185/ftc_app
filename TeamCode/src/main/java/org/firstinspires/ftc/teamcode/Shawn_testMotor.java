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
import com.qualcomm.robotcore.util.Range;

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
    final int MIN_GB_TICKS = 383;
    int GBTicks = 0;



    @Override
    public void init() {

        Shawn.init(hardwareMap, false);

        GBTicks = Shawn.leftRear.getCurrentPosition();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Initial Ticks", GBTicks);
        telemetry.update();

        Shawn.leftRear.setTargetPosition(GBTicks);
        Shawn.leftRear.setPower(1);

    }

    @Override
    public void loop() {


        if (gamepad1.x && GBTicks < MAX_GB_TICKS) {
            GBTicks += GB_TICKS_PER_ROTATION/SUB_ROTATION;
        }

        if (gamepad1.y && GBTicks > MIN_GB_TICKS) {
            GBTicks -= GB_TICKS_PER_ROTATION/SUB_ROTATION;
        }

        Shawn.leftRear.setTargetPosition(GBTicks);
//        while(Shawn.leftRear.isBusy()){}
        telemetry.addData("Ticks set to", GBTicks);
        telemetry.addData("Ticks Reached", Shawn.leftRear.getCurrentPosition());
        telemetry.update();

//        if (gamepad1.dpad_up) {
//            Shawn.leftRear.setPower(1);
//        } else if (gamepad1.dpad_down) {
//            Shawn.leftRear.setPower(-1);
//        } else {
//            Shawn.leftRear.setPower(0);
//        }

//        if(gamepad1.x) {
//            Shawn.hub1Motor.setPower(1);
//        } else {
//            Shawn.hub1Motor.setPower(0);
//        }
//
//        if(gamepad1.y) {
//            Shawn.hub2Motor.setPower(1);
//        } else {
//            Shawn.hub2Motor.setPower(0);
//        }

    }

}
