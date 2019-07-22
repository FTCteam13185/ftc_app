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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Basic: League Test Drive (No encoder)", group="Iterative Opmode")
public class LeagueTestThing extends OpMode {

    // Declare OpMode members.
    private DcMotor leftFront = null;   // white - port 2
    private DcMotor leftRear = null;    // yellow - port 1
    private DcMotor rightFront = null;  // green - port 3
    private DcMotor rightRear = null;   // blue - port 0

    private Servo clawServo = null;
    private CRServo armMotor = null;

    private DigitalChannel armStop = null;

    private double clawPos;
    private final double clawIncrement = 0.02;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");

        // servos
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        armMotor = hardwareMap.get(CRServo.class, "armMotor");

        clawPos = clawServo.getPosition();

        // arm touch sensor
        armStop = hardwareMap.get(DigitalChannel.class, "armStop");
        armStop.setMode(DigitalChannel.Mode.INPUT);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        clawServo.setDirection(Servo.Direction.FORWARD);
        armMotor.setDirection(CRServo.Direction.FORWARD);

        // while the power of the wheels is 0, brake
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    @Override
    public void loop() {
        // Setup a variable for each side of the robot
        double leftPower;
        double rightPower;

        // This driving mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
//        double drive = -gamepad1.left_stick_y;
//        double turn  =  gamepad1.right_stick_x;
//        leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
//        rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;
//
//        // Send calculated power to wheels
//        leftFront.setPower(leftPower);
//        leftRear.setPower(leftPower);
//        rightFront.setPower(rightPower);
//        rightRear.setPower(rightPower);

        // strafing control

        double control = 2;

        if (gamepad1.right_bumper) {
            control = 4;
        } else if (gamepad1.left_bumper) {
            control = 1;
        }

        double driveY = -gamepad1.left_stick_y;
        double driveX = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

//        double leftFrontDrive = Range.clip(driveY + driveX, -1.0, 1.0);
//        double leftRearDrive = Range.clip(driveY - driveX, -1.0, 1.0);
//        double rightFrontDrive = Range.clip(driveY - driveX, -1.0, 1.0);
//        double rightRearDrive = Range.clip(driveY + driveX, -1.0, 1.0);
//
//        double leftFrontPower = Range.clip(leftFrontDrive + turn, -1.0, 1.0);
//        double leftRearPower = Range.clip(leftRearDrive + turn, -1.0, 1.0);
//        double rightFrontPower = Range.clip(rightFrontDrive - turn, -1.0, 1.0);
//        double rightRearPower = Range.clip(rightRearDrive - turn, -1.0, 1.0);

        // ^ condensed
        double leftFrontPower = Range.clip((driveY + driveX) + turn, -1.0, 1.0);
        double leftRearPower = Range.clip((driveY - driveX) + turn, -1.0, 1.0);
        double rightFrontPower = Range.clip((driveY - driveX) - turn, -1.0, 1.0);
        double rightRearPower = Range.clip((driveY + driveX) - turn, -1.0, 1.0);

        leftFront.setPower(leftFrontPower / control);
        leftRear.setPower(leftRearPower / control);
        rightFront.setPower(rightFrontPower / control);
        rightRear.setPower(rightRearPower / control);

        // controlling the claw
        if (gamepad1.dpad_right) {
            clawPos = Range.clip(clawPos + clawIncrement, -1.0, 1.0);
        } else if (gamepad1.dpad_left) {
            clawPos = Range.clip(clawPos - clawIncrement, -1.0, 1.0);
        }
        clawServo.setPosition(clawPos);

        // control arm
        if (gamepad1.dpad_up) {
            armMotor.setPower(0.5);
        } else if (gamepad1.dpad_down && armStop.getState()) {
            armMotor.setPower(-0.5);
        } else {
            armMotor.setPower(0);
        }


        // write touch sensor control to screen
        telemetry.addData("Touch Sensor", armStop.getState());

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        // stop all motion
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }

}
