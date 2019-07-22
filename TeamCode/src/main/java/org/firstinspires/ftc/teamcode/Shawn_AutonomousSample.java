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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static java.lang.Math.abs;

/**
 * This file illustrates the concept of driving a path based on Gyro heading and encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the Shawn.
 * The code is structured as a LinearOpMode
 * <p>
 * The code REQUIRES that you DO have encoders on the wheels,
 * otherwise you would use: PushbotAutoDriveByTime;
 * <p>
 * This code ALSO requires that you have a Modern Robotics I2C gyro with the name "gyro"
 * otherwise you would use: PushbotAutoDriveByEncoder;
 * <p>
 * This code requires that the drive Motors have been configured such that a positive
 * power command moves them forward, and causes the encoders to count UP.
 * <p>
 * This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 * <p>
 * In order to calibrate the Gyro correctly, the Shawn must remain stationary during calibration.
 * This is performed when the INIT button is pressed on the Driver Station.
 * This code assumes that the Shawn is stationary when the INIT button is pressed.
 * If this is not the case, then the INIT should be performed again.
 * <p>
 * Note: in this example, all angles are referenced to the initial coordinate frame set during the
 * the Gyro Calibration process, or whenever the program issues a resetZAxisIntegrator() call on the Gyro.
 * <p>
 * The angle of movement/rotation is assumed to be a standardized rotation around the Shawn Z axis,
 * which means that a Positive rotation is Counter Clock Wise, looking down on the field.
 * This is consistent with the FTC field coordinate conventions set out in the document:
 * ftc_app\doc\tutorial\FTC_FieldCoordinateSystemDefinition.pdf
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name = "Shawn: Autonomous Unlatch and SAMPLE", group = "Pushbot")
//@Disabled
public class Shawn_AutonomousSample extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareShawn Shawn = new HardwareShawn();   // Use a Shawn's hardware

    static final double COUNTS_PER_MOTOR_REV = 537.6;    // goBILDA motor
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.141592653589793238462643383279);

    final double TICKS_PER_GB_ROTATION = 537.6;
    final int MAX_GB_TICKS = 9600 - (int) (TICKS_PER_GB_ROTATION * 1);
    final int MIN_GB_TICKS = 0;

    // These constants define the desired driving/controlType characteristics
    // They can/should be tweaked to suite the specific Shawn drive train.
    static final double DRIVE_SPEED = 0.7;     // Nominal speed for better accuracy.
    static final double TURN_SPEED = 0.6;     // Nominal half speed for better accuracy.
    static final double STRAFE_SPEED = 0.4;   // Slow for to go straighter

    static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.06;    // Larger is more responsive, but also less stable

    public static final int TICKS_PER_DEGREE = 4;
    public static final double ARM_POWER = 1;

  // Constant for Sampling path
    public static final double TILE_DIAGONAL = 2 * Math.sqrt(2) * 12;

    public static final double DETECT_BACKUP = /*5.5*/ 0;

    public static final double DIST_TO_TILE_CORNER = 14;
    public static final double DIST_TO_CENTER_MINERAL = DIST_TO_TILE_CORNER + (TILE_DIAGONAL / 2) + DETECT_BACKUP;
    public static final double DIST_TO_SIDE_MINERAL = Math.sqrt(Math.pow(DIST_TO_CENTER_MINERAL, 2) + Math.pow(TILE_DIAGONAL/2, 2));
    public static final double ANGLE_TO_SIDE_MINERAL = (Math.acos(DIST_TO_CENTER_MINERAL/DIST_TO_SIDE_MINERAL) * 180) / Math.PI;

    // so we do not go to far when we go to knock off the mineral (distance to center mineral is the distance from the middle of the robot)
    public static final double CUBE_SIDE = 2;
    public static final double LESS = 9 - (2 * CUBE_SIDE);

    // The hypotenuse of the triangle we make when we turn to knock off a gold not in the center
    public static final double FORWARD_SIDE = DIST_TO_SIDE_MINERAL - LESS;
    public static final double FORWARD_CENTER = DIST_TO_CENTER_MINERAL - LESS;
    public static final double SIDE_A = DIST_TO_TILE_CORNER + DETECT_BACKUP;
    public static final double BACK_TO_WALL = -4.00 * 12;
    public static final double DETECT_TIME = 1.5;

    // SOUND SOUND SOUND SOUND SOUND SOUND SOUND SOUND SOUND SOUND SOUND SOUND SOUND SOUND SOUND SOUND SOUND
    MediaPlayer lightsaber = null;
    MediaPlayer march = null;

    @Override
    public void runOpMode() throws InterruptedException {

        /*
         * Initialize the standard drive system variables.
         * The init() method of the hardware class does most of the work here
         */
        Shawn.init(hardwareMap, true);

        // TFOD TFOD TFOD TFOD TFOD TFOD TFOD TFOD TFOD TFOD TFOD TFOD TFOD TFOD TFOD TFOD TFOD TFOD TFOD
        Shawn_TensorDetectionCLASS tfod = new Shawn_TensorDetectionCLASS(this, this.hardwareMap, 1100);

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            tfod.init();
        } else {
            telemetry.addLine("uh oh spaghetti-o! This device does not support TFOD!!");
        }

        // SOUND SOUND SOUND
        lightsaber = MediaPlayer.create(this.hardwareMap.appContext, R.raw.lightsaber);
        lightsaber.seekTo(0);

        march = MediaPlayer.create(this.hardwareMap.appContext, R.raw.imperialmarch);
        march.seekTo(9350);

        // Ensure the Shawn it stationary, then reset the encoders and calibrate the gyro.
        Shawn.leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Shawn.rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Shawn.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Shawn.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Shawn.leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        Shawn.rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        Shawn.leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        Shawn.rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        Shawn.actuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Shawn.actuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Shawn.armRotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Shawn.armRotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Shawn.leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Shawn.rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Shawn.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Shawn.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // BREADY BREADY BREADY BREADY BREADY BREADY BREADY BREADY BREADY BREADY BREADY BREADY BREADY BREADY BREADY BREADY BREADY BREADY BREADY BREADY BREADY BREADY BREADY

        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {
            Thread.sleep(10);
            telemetry.addData(">", "Robot Heading = %f", Shawn.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            telemetry.addData(">", "Heading Error = %f", getError(0));
            telemetry.update();
        }
        waitForStart();

        //   relicTrackables.activate();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // Put a hold after each turn

        //fffsdf

        // UNLOCK AND LOWER UNLOCK AND LOWER UNLOCK AND LOWER UNLOCK AND LOWER UNLOCK AND LOWER UNLOCK AND LOWER UNLOCK AND LOWER

        // raising actuator
        Shawn.actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Shawn.actuator.setTargetPosition(MAX_GB_TICKS);
        Shawn.actuator.setPower(1);

        // playing lightsaber noise
        lightsaber.start();

        // wait until actuator reaches final position
        while(Shawn.actuator.isBusy()) {}

        // stop actuator
        Shawn.actuator.setPower(0);

        march.start();          //march start

        // MOVING MOVING MOVING MOVING MOVING MOVING MOVING MOVING MOVING MOVING MOVING MOVING MOVING MOVING MOVING MOVING MOVING\

        // strafe away from the lander
      //  gyroStrafe(STRAFE_SPEED, 5, 2);

        // Turn 180 degrees and back up towards lander
        gyroTurn(TURN_SPEED, 180);
        gyroHold(TURN_SPEED, 180, 0.5);

        // lowering actuator
        Shawn.actuator.setTargetPosition(MIN_GB_TICKS);
        Shawn.actuator.setPower(1);

        // detect backup
        gyroDrive(DRIVE_SPEED, -DETECT_BACKUP, 180);
        gyroHold(TURN_SPEED, 180, 0.1);

        // detect and knock off gold

        // sdfsdf // ctrl + f and type to quickly find this position
        double sideB = SIDE_A * (Math.tan(((ANGLE_TO_SIDE_MINERAL * Math.PI) / 180)));
        double hypotenuse = Math.sqrt(Math.pow(SIDE_A, 2) + Math.pow(sideB, 2));

        if (tfod.detectGold(DETECT_TIME)) {
            telemetry.addLine("THE GOLD IS IN THE CENTER!!! WHOO");
            // knock gold
            gyroDrive(DRIVE_SPEED, FORWARD_CENTER, 180);
            gyroHold(TURN_SPEED, 180, 0.1);
            // drive back
            gyroDrive(DRIVE_SPEED, (-FORWARD_CENTER) + SIDE_A, 180);
            gyroHold(TURN_SPEED, 180, 0.1);
        } else {
            // turn to look at left mineral
            gyroTurn(TURN_SPEED, 180 + ANGLE_TO_SIDE_MINERAL);
            gyroHold(TURN_SPEED, 180 + ANGLE_TO_SIDE_MINERAL, 0.2);
            if (tfod.detectGold(DETECT_TIME)) {
                telemetry.addLine("THE GOLD IS ON THE LEFT!!");
                // knock gold
                gyroDrive(DRIVE_SPEED, FORWARD_SIDE, 180 + ANGLE_TO_SIDE_MINERAL);
                gyroHold(TURN_SPEED, 180 + ANGLE_TO_SIDE_MINERAL, 0.1);
                // drive back
                gyroDrive(DRIVE_SPEED, (-FORWARD_SIDE) + hypotenuse, 180 + ANGLE_TO_SIDE_MINERAL);
                gyroHold(TURN_SPEED, 180 + ANGLE_TO_SIDE_MINERAL, 0.1);
            } else {
                telemetry.addLine("THE GOLD SHOULD BE ON THE RIGHT!!");
                // turn to face gold
                gyroTurn(TURN_SPEED, 180 - ANGLE_TO_SIDE_MINERAL);
                gyroHold(TURN_SPEED, 180 - ANGLE_TO_SIDE_MINERAL, 0.2);
                // knock gold
                gyroDrive(DRIVE_SPEED, FORWARD_SIDE, 180 - ANGLE_TO_SIDE_MINERAL);
                gyroHold(TURN_SPEED, 180 - ANGLE_TO_SIDE_MINERAL, 0.1);
                // drive back
                gyroDrive(DRIVE_SPEED, (-FORWARD_SIDE) + hypotenuse, 180 - ANGLE_TO_SIDE_MINERAL);
                gyroHold(TURN_SPEED, 180 - ANGLE_TO_SIDE_MINERAL, 0.1);
            }
        }
        telemetry.update();

        // set actuator power to 0
        Shawn.actuator.setPower(0);

        while (march.getCurrentPosition() < 23700) {}
        march.pause();

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }


    /*
     * Method to drive on a fixed compass bearing (angle), based on encoder counts.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the desired armPosition
     * 2) Driver stops the opmode running.
     *
     * @param speed    Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance Distance (in inches) to move from current armPosition.  Negative distance means move backwards.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     */
    public void gyroDrive(double speed, double distance, double angle) {                                                // GYRO DRIVE GYRO DRIVE GYRO DRIVE

        int newLeftRearTarget;
        int newLeftFrontTarget;
        int newRightRearTarget;
        int newRightFrontTarget;
        int moveCounts;
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target armPosition, and pass to motor controller
            moveCounts = (int) (distance * COUNTS_PER_INCH);
            newLeftRearTarget = Shawn.leftRear.getCurrentPosition() + moveCounts;
            newLeftFrontTarget = Shawn.leftFront.getCurrentPosition() + moveCounts;
            newRightRearTarget = Shawn.rightRear.getCurrentPosition() + moveCounts;
            newRightFrontTarget = Shawn.rightFront.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            Shawn.leftRear.setTargetPosition(newLeftRearTarget);
            Shawn.leftFront.setTargetPosition(newLeftFrontTarget);
            Shawn.rightRear.setTargetPosition(newRightRearTarget);
            Shawn.rightFront.setTargetPosition(newRightFrontTarget);

            Shawn.leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Shawn.rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Shawn.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Shawn.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    //        Shawn.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // start motion.
            speed = Range.clip(abs(speed), 0.0, 1.0);
            Shawn.leftRear.setPower(speed);
            Shawn.rightRear.setPower(speed);
            Shawn.leftFront.setPower(speed);
            Shawn.rightFront.setPower(speed);

            // keep looping while we are still active, and ALL motors are running.
            while (opModeIsActive() &&
                    (Shawn.leftRear.isBusy() && Shawn.rightRear.isBusy() &&
                            Shawn.leftFront.isBusy()  && Shawn.rightFront.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0) {
                    steer *= -1.0;
             //       Shawn.rightFront.setDirection(DcMotorSimple.Direction.REVERSE);             // **** TAKE OUT WHEN WE HAVE THE GOOD MOTOR
                } else {
             //       Shawn.rightFront.setDirection(DcMotorSimple.Direction.FORWARD);             // **** TAKE OUT WHEN WE HAVE THE GOOD MOTOR
                }

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(abs(leftSpeed), abs(rightSpeed));
                if (max > 1.0) {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                Shawn.leftRear.setPower(leftSpeed);
                Shawn.rightRear.setPower(rightSpeed);
                Shawn.leftFront.setPower(leftSpeed);
                Shawn.rightFront.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                telemetry.addData("Target", "%7d:%7d", newLeftRearTarget, newRightRearTarget);
                telemetry.addData("Actual", "%7d:%7d", Shawn.leftRear.getCurrentPosition(),
                        Shawn.rightRear.getCurrentPosition());
                telemetry.addData("L&R Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            Shawn.leftRear.setPower(0);
            Shawn.rightRear.setPower(0);
            Shawn.leftFront.setPower(0);
            Shawn.rightFront.setPower(0);

            // Turn off RUN_TO_POSITION
            Shawn.leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Shawn.rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            Shawn.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //Shawn.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            Shawn.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Reset the rightFront wheel direction back to "Forward" before exiting.
            //Shawn.rightFront.setDirection(DcMotorSimple.Direction.FORWARD);             // **** TAKE OUT WHEN WE HAVE THE GOOD MOTOR

        }
    }

    /**
     * Method to spin on central axis to point in a new direction.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the heading (angle)
     * 2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle Absolute Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn(double speed, double angle) {                                                                      // GYRO TURN GYRO TURN GYRO TURN
        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    public void gyroStrafe(double speed, double distance, double angle) {                                                   // GYRO STRAFE GYRO STRAFE GYRO STRAFE GYRO STRAFE

        int newLeftRearTarget;
        int newLeftFrontTarget;
        int newRightRearTarget;
        int newRightFrontTarget;
        int moveCounts;
        int revMoveCounts;
        double max;
        double error;
        double steer;
        double lf, lr, rr, rf;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target armPosition, and pass to motor controller
            //     moveCounts = (int) (distance * COUNTS_PER_INCH);
            moveCounts = (int) (distance * COUNTS_PER_MOTOR_REV);
            revMoveCounts = -moveCounts;
            if (distance < 0) {
                // right
                newLeftRearTarget = Shawn.leftFront.getCurrentPosition() - moveCounts;
                newLeftFrontTarget = Shawn.rightFront.getCurrentPosition() + moveCounts;
                newRightRearTarget = Shawn.leftRear.getCurrentPosition() + moveCounts;
                newRightFrontTarget = Shawn.rightRear.getCurrentPosition() - moveCounts;
            } else {
                // left
                newLeftRearTarget = Shawn.leftFront.getCurrentPosition() + moveCounts;
                newLeftFrontTarget = Shawn.rightFront.getCurrentPosition() - moveCounts;
                newRightRearTarget = Shawn.leftRear.getCurrentPosition() - moveCounts;
                newRightFrontTarget = Shawn.rightRear.getCurrentPosition() + moveCounts;
            }

            // Set Target and Turn On RUN_TO_POSITION
            Shawn.leftRear.setTargetPosition(newLeftRearTarget);
            Shawn.leftFront.setTargetPosition(newLeftFrontTarget);
            Shawn.rightRear.setTargetPosition(newRightRearTarget);
            Shawn.rightFront.setTargetPosition(newRightFrontTarget);

            Shawn.leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Shawn.rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Shawn.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Shawn.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //        Shawn.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // start motion.
            speed = Range.clip(abs(speed), 0.0, 1.0);
            if (distance < 0) {
                Shawn.leftFront.setPower(speed);
                Shawn.rightFront.setPower(speed);
                Shawn.leftRear.setPower(speed);
                Shawn.rightRear.setPower(speed);
            }

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (Shawn.leftRear.isBusy() && Shawn.rightRear.isBusy() &&
                            Shawn.leftFront.isBusy()  && Shawn.rightFront.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0) {
                    steer *= -1.0;
                }

                lf = speed + steer;
                rf = speed + steer;
                lr = speed - steer;
                rr = speed - steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.max(abs(lf), abs(rf)), Math.max(abs(lr), abs(rr)));
                if (max > 1.0) {
                    lr /= max;
                    rf /= max;
                    lr /= max;
                    rr /= max;
                }

                Shawn.leftFront.setPower(lf);
                Shawn.rightFront.setPower(rf);
                Shawn.leftRear.setPower(lr);
                Shawn.rightRear.setPower(rr);

                // Display drive status for the driver.
                telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                telemetry.addData("Target", "%7d:%7d", newLeftRearTarget, newRightRearTarget);
                telemetry.addData("Actual", "%7d:%7d", Shawn.leftRear.getCurrentPosition(),
                        Shawn.rightRear.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            Shawn.leftRear.setPower(0);
            Shawn.rightRear.setPower(0);
            Shawn.leftFront.setPower(0);
            Shawn.rightFront.setPower(0);

            // Turn off RUN_TO_POSITION
            Shawn.leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Shawn.rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Shawn.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Shawn.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

    /**
     * Method to obtain & hold a heading for a finite amount of time
     * Move will stop once the requested time has elapsed
     *
     * @param speed    Desired speed of turn.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     * @param holdTime Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold(double speed, double angle, double holdTime) {                                                     // GYRO HOLD GYRO HOLD GRYO HOLD GYRO HOLD

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        Shawn.leftRear.setPower(0);
        Shawn.rightRear.setPower(0);
        Shawn.leftFront.setPower(0);
        Shawn.rightFront.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading controlType.
     *
     * @param speed  Desired speed of turn.
     * @param angle  Absolute Angle (in Degrees) relative to last gyro reset.
     *               0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *               If a relative angle is required, add/subtract from current heading.
     * @param PCoeff Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {                                                          // ON HEADING ON HEADING ON HEADING ON HEADING ON HEADING
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * steer;
            leftSpeed = -rightSpeed;
        }

        // Send desired speeds to motors.
        Shawn.leftRear.setPower(leftSpeed);
        Shawn.rightRear.setPower(rightSpeed);
        Shawn.leftFront.setPower(leftSpeed);
        Shawn.rightFront.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the Shawn's current heading
     *
     * @param targetAngle Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the Shawn's frame of reference
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