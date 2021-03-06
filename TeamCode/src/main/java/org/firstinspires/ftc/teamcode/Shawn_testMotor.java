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

import android.media.MediaPlayer;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static java.lang.Math.abs;


@TeleOp(name = "Test: TestMotor", group = "Motors")
//@Disabled
public class Shawn_testMotor extends OpMode {

    /* Declare OpMode members. */

//    public static final double ARM_INCREMENT
//    public static final double CLAW_INCREMENT     = 0.03;
//    public static final int LOOP_WAIT             = 5;
//    public static final double rightStrafeControl = 0.875;
//    public static final double leftStrafeControl  = 0.885;

    HardwareShawn Shawn = new HardwareShawn();   // Use Shawn's hardware

    //   SoundPlayer sPlayer = new SoundPlayer(1,256*1024);

    boolean drive = true;

    double initServoPos;

    final int ticksPerRotation = 1440;

    double armPosition = 1;
    double clawPosition;

    int numLoops = 0;

    boolean controlType = true;

    static final double P_DRIVE_COEFF = 0.05;    // Larger is more responsive, but also less stable
    static final double DRIVE_SPEED = 0.5;       // SPEEDING AWAY
    final int GB_TICKS_PER_ROTATION = 384;
    final int REV_TICKS_PER_ROTATION = 2240;
    final int SUB_ROTATION = 4;
    final int MAX_GB_TICKS = 9600 - (int) (GB_TICKS_PER_ROTATION * 1);
    final int MIN_GB_TICKS = 0;

    int GBTicks = 0;

    int armTicks = 0;
    int harvesterTicks = 0;
    boolean holdHarvester;
    boolean holdArm;

    int sweep = 0;

    double lr = 0;
    double rr = 0;
    double lf = 0;
    double rf = 0;

    boolean temp;

    float currAngle = 0;

    boolean saveAngle = true;
    float lastAngle = 0;

    int armPos = 0;

    // SOUND STUFF
    MediaPlayer lightsaber = null;

    @Override
    public void init() {

        // CURRENTLY SET GYRO TO TRUE! ONLY KEEP IF GYRO STAYS
        Shawn.init(hardwareMap, true);

        Shawn.actuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Shawn.actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        GBTicks = Shawn.actuator.getCurrentPosition();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Initial Ticks", GBTicks);
        telemetry.update();

//        Shawn.actuator.setTargetPosition(GBTicks);
//        Shawn.actuator.setPower(1);


        //SOUND STUFF
        lightsaber = MediaPlayer.create(this.hardwareMap.appContext, R.raw.lightsaber);
        lightsaber.seekTo(0);

   //    armTicks = Shawn.armRotation.getCurrentPosition();
    //    harvesterTicks = Shawn.harvester.getCurrentPosition();
        holdHarvester = true;
        holdArm = true;

        temp = true;
    }

    @Override
    public void loop() {

        // GYRO GYRO GYRO GYRO GYRO GYRO GYRO GYRO GYRO GYRO GYRO GYRO GYRO GYRO
        Orientation angles = Shawn.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currAngle = angles.firstAngle;

        double error = 0;
        double steer = 0;
        double maxSpeed = 0;

        double control = 1;

        lr = 0;
        rr = 0;
        lf = 0;
        rf = 0;

        //ACTUATOR ACTUATOR ACTUATOR ACTUATOR ACTUATOR ACTUATOR ACTUATOR ACTUATOR ACTUATOR ACTUATOR ACTUATOR
        if (gamepad1.y) {
            GBTicks = MAX_GB_TICKS;
            if (!lightsaber.isPlaying()) {
                lightsaber.seekTo(0);
                lightsaber.start();
            }
        } else if (gamepad1.a) {
            GBTicks = MIN_GB_TICKS;
        } else if (gamepad1.dpad_down) {
            GBTicks -= 10;
        }else if (gamepad1.dpad_up) {
            GBTicks += 10;
        }

        Shawn.actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Shawn.actuator.setTargetPosition(GBTicks);
        Shawn.actuator.setPower(1);

        //SWEEPY THINGY SWEEPY THINGY SWEEPY THINGY SWEEPY THINGY

        if (gamepad2.dpad_up) {
            Shawn.sweepy.setPower(0.75);
        } else if (gamepad2.dpad_down) {
            Shawn.sweepy.setPower(-0.75);
        } else {
            Shawn.sweepy.setPower(0);
        }

        // ARM CONTROL ARM CONTROL ARM CONTROL ARM CONTROL ARM CONTROL

        if (gamepad2.b) {
            Shawn.armRotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Shawn.armRotation.setTargetPosition(-1690);
            Shawn.armRotation.setPower(0.2);
            armPos = 2;
        } else if (gamepad2.y) {
            Shawn.armRotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Shawn.armRotation.setTargetPosition(-3500);
            Shawn.armRotation.setPower(0.3);
            armPos = 3;
        }

        if (armPos != 0) {
            if (!Shawn.armRotation.isBusy()) {
                armPos = 0;
                Shawn.armRotation.setPower(0);
            }
        }

        if (-gamepad2.left_stick_y > 0.5 && armPos == 0) {
            if (Shawn.armRotation.getCurrentPosition() > -3560) {
                Shawn.armRotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Shawn.armRotation.setPower(-0.25);
            }
            else {
                Shawn.armRotation.setPower(0);
            }
        } else if (-gamepad2.left_stick_y < -0.5 && armPos == 0) {
            if (Shawn.armRotation.getCurrentPosition() < 0) {
                Shawn.armRotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Shawn.armRotation.setPower(0.25);
            }
            else {
                Shawn.armRotation.setPower(0);
            }
        } else if (armPos == 0) {
            Shawn.armRotation.setPower(0);
        }
//        Shawn.armRotation.setTargetPosition(armTicks);
//        Shawn.armRotation.setPower(1);

        // PULLEY CONTROL PULLEY CONTROL PULLEY CONTROL PULLEY CONTROL PULLEY CONTROL PULLEY CONTROL

        if (-gamepad2.right_stick_y > 0.1) {
            Shawn.pulley.setPower(-gamepad2.right_stick_y);
        } else if (-gamepad2.right_stick_y < -0.1) {
            Shawn.pulley.setPower(-gamepad2.right_stick_y);
        } else {
            Shawn.pulley.setPower(0);
        }

//        if (-gamepad2.right_stick_y > 0.1) {
//            Shawn.brakePulley.setPower(-gamepad2.right_stick_y / 2);
//        } else if (-gamepad2.right_stick_y < -0.1) {
//            Shawn.brakePulley.setPower(-gamepad2.right_stick_y / 2);
//        } else {
//            Shawn.brakePulley.setPower(0);
//        }
//        telemetry.addData("pulley ticks: ", Shawn.brakePulley.getCurrentPosition());
//        telemetry.update();

        //DRIVE DRIVE DRIVE DRIVE DRIVE DRIVE DRIVE DRIVE DRIVE DRIVE
        if (Math.abs(gamepad1.left_stick_y) > 0.1) {
            saveAngle = true;
            lr = -gamepad1.left_stick_y + (gamepad1.left_stick_x);
            lf = -gamepad1.left_stick_y + (gamepad1.left_stick_x);
            rr = -gamepad1.left_stick_y + (-gamepad1.left_stick_x);
            rf = -gamepad1.left_stick_y + (-gamepad1.left_stick_x);

        } else if (Math.abs(gamepad1.right_stick_x) > 0.1) {
            saveAngle = true;
            lr = gamepad1.right_stick_x;
            lf = gamepad1.right_stick_x;
            rr = -gamepad1.right_stick_x;
            rf = -gamepad1.right_stick_x;

            // STRAFE STRAFE STRAFE STRAFE STRAFE STRAFE STRAFE STRAFE STRAFE STRAFE STRAFE STRAFE STRAFE STRAFE STRAFE STRAFE
        } else if (gamepad1.right_trigger != 0) {
            if (saveAngle) {
                lastAngle = Shawn.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                saveAngle = false;
            }
        error = getError(lastAngle);
        steer = getSteer(error, P_DRIVE_COEFF);

        lf =  DRIVE_SPEED + steer;  // CHANGED STEER
        rf = -DRIVE_SPEED - steer;
        lr = -DRIVE_SPEED + steer;
        rr =  DRIVE_SPEED - steer;

        // Normalize speeds if either one exceeds +/- 1.0;
        maxSpeed = Math.max(Math.max(abs(lf), abs(rf)), Math.max(abs(lr), abs(rr)));
        if (maxSpeed > 1.0) {
            lr /= maxSpeed;
            rf /= maxSpeed;
            lr /= maxSpeed;
            rr /= maxSpeed;
        }

        } else if (gamepad1.left_trigger != 0) {
            if (saveAngle) {
                lastAngle = Shawn.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                saveAngle = false;
            }

            error = getError(lastAngle);
            steer = getSteer(error, P_DRIVE_COEFF);

            lf = -DRIVE_SPEED + steer;  // CHANGED STEER
            rf =  DRIVE_SPEED - steer;
            lr =  DRIVE_SPEED + steer;
            rr = -DRIVE_SPEED - steer;

            // Normalize speeds if either one exceeds +/- 1.0;
            maxSpeed = Math.max(Math.max(abs(lf), abs(rf)), Math.max(abs(lr), abs(rr)));
            if (maxSpeed > 1.0) {
                lr /= maxSpeed;
                rf /= maxSpeed;
                lr /= maxSpeed;
                rr /= maxSpeed;
            }

        } else if (gamepad2.right_trigger != 0) {
            lr = gamepad2.right_trigger;
            lf = gamepad2.right_trigger;
            rr = -gamepad2.right_trigger;
            rf = -gamepad2.right_trigger;
        } else if (gamepad2.left_trigger != 0) {
            lr = -gamepad2.left_trigger;
            lf = -gamepad2.left_trigger;
            rr = gamepad2.left_trigger;
            rf = gamepad2.left_trigger;
        } else {
                saveAngle = true;
        }



        // FINE CONTROL FINE CONTROL FINE CONTROL FINE CONTROL FINE CONTROL FINE CONTROL
        if (gamepad1.left_bumper) {
            control = 0.5;
        } else if (gamepad1.right_bumper) {
            control = 2;
        }

        Shawn.leftRear.setPower((lr / 2) / control);
        Shawn.leftFront.setPower((lf / 2) / control);
        Shawn.rightRear.setPower((rr / 2) / control);
        Shawn.rightFront.setPower((rf / 2) / control);

        telemetry.update();

    }

    /**
     * getError determines the error between the target angle and the Shawn's current heading
     *
     * @param targetAngle Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on Shawn's frame of reference
     * +ve error means the Shawn should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {                                                                                // GET ERROR GET ERROR GET ERROR GET ERROR

        double robotError;

        // calculate error in -179 to +180 range  (
        Orientation angles = Shawn.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        robotError = targetAngle - angles.firstAngle;
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     *
     * @param error  Error angle in Shawn relative degrees
     * @param PCoeff Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {                                                                        // GET STEER GET STEER GET STEER GET STEER
        return Range.clip(error * PCoeff, -1, 1);
    }

}
