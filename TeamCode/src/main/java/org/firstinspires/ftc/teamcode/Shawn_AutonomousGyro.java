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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static java.lang.Math.abs;

/**
 * This file illustrates the concept of driving a path based on Gyro heading and encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the Shawn.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that you have a Modern Robotics I2C gyro with the name "gyro"
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *  This code requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 *  In order to calibrate the Gyro correctly, the Shawn must remain stationary during calibration.
 *  This is performed when the INIT button is pressed on the Driver Station.
 *  This code assumes that the Shawn is stationary when the INIT button is pressed.
 *  If this is not the case, then the INIT should be performed again.
 *
 *  Note: in this example, all angles are referenced to the initial coordinate frame set during the
 *  the Gyro Calibration process, or whenever the program issues a resetZAxisIntegrator() call on the Gyro.
 *
 *  The angle of movement/rotation is assumed to be a standardized rotation around the Shawn Z axis,
 *  which means that a Positive rotation is Counter Clock Wise, looking down on the field.
 *  This is consistent with the FTC field coordinate conventions set out in the document:
 *  ftc_app\doc\tutorial\FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Shawn: Auto Drive By Gyro", group="Pushbot")
//@Disabled
public class Shawn_AutonomousGyro extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareShawn Shawn = new HardwareShawn();   // Use a Shawn's hardware

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suite the specific Shawn drive train.
    static final double     DRIVE_SPEED             = 0.6;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.5;     // Nominal half speed for better accuracy.

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.06;    // Larger is more responsive, but also less stable

    public static final int TICKS_PER_DEGREE    = 4;
    public static final double ARM_POWER        = 1;

    Shawn_SensorMRColor sensor = new Shawn_SensorMRColor();

    @Override
    public void runOpMode() throws InterruptedException {

        /*
         * Initialize the standard drive system variables.
         * The init() method of the hardware class does most of the work here
         */
        Shawn.init(hardwareMap);

        // Ensure the Shawn it stationary, then reset the encoders and calibrate the gyro.
        Shawn.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Shawn.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();



//        Shawn.imu.calibrate();

        // make sure the gyro is calibrated before continuing
//        while (!isStopRequested() && Shawn.gyro.isCalibrating())  {
//            sleep(50);
//            idle();
//        }

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

        Shawn.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Shawn.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {
            Thread.sleep(10);
            telemetry.addData(">", "Robot Heading = %f", Shawn.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            telemetry.update();
        }

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // Put a hold after each turn

        //fff

//        gyroDrive(DRIVE_SPEED/1.5, 3.5, 0.0);    // drive to position
//
//        // let tail down
//        moveArm(40, 0, ARM_POWER);
//        Thread.sleep(1000);
//        moveArm(-50, 0, ARM_POWER/4);
//        Thread.sleep(500);
////        gyroHold(TURN_SPEED, 0, 2);
////        for (int i = 0; i < 5; i++) {
////            moveArm(-10, 0, ARM_POWER);
////     //       gyroHold(TURN_SPEED, 0, 0.2);
////            Thread.sleep(200);
////        }
//        Shawn.armShoulder.setPower(0);
//        Shawn.armShoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        // boop code
////        if (sensor.isBlue(Shawn.colorSensor)) {
////            telemetry.clearAll();
////            telemetry.addLine("The ball is blue");
////            telemetry.update();
////            Thread.sleep(1500);
////            gyroTurn(TURN_SPEED, 20);
////            gyroHold(TURN_SPEED, 20, 0.5);
////            gyroTurn(TURN_SPEED, 0);
////            gyroHold(TURN_SPEED, 0, 0.5);
////        } else {
////            telemetry.clearAll();
////            telemetry.addLine("The ball is red");
////            telemetry.update();
////            Thread.sleep(1500);
////            gyroTurn(TURN_SPEED, -20);
////            gyroHold(TURN_SPEED, -20, 0.5);
////            gyroTurn(TURN_SPEED, 0);
////            gyroHold(TURN_SPEED, 0, 0.5);
////        }
//
//        int initElbowPos = Shawn.armElbow.getCurrentPosition();
//
//        moveArm(0, 140, ARM_POWER/1.5);
//        moveArm(0, 150, ARM_POWER/4);
//        telemetry.addLine("moving elbow up 120deg");
//        telemetry.update();
//        Thread.sleep(1000);
////        gyroHold(TURN_SPEED, 0, 1);
////        for (int i = 0; i < 20; i++) {
////            moveArm(0, 10, ARM_POWER);
////      //      gyroHold(TURN_SPEED, 0, 0.3);
////            Thread.sleep(300);
////        }
//
//        gyroDrive(DRIVE_SPEED/1.5, -3, 0);
//
//        Thread.sleep(10000);
//
//        gyroTurn(TURN_SPEED, 10);
//        gyroHold(TURN_SPEED, 10, 0.5);
//        gyroTurn(TURN_SPEED, 0);
//        gyroHold(TURN_SPEED, 0, 0.5);
//        gyroTurn(TURN_SPEED, -10);
//        gyroHold(TURN_SPEED, -10, 0.5);
//        gyroTurn(TURN_SPEED, 0);
//        gyroHold(TURN_SPEED, 0, 0.5);
//
//        gyroDrive(DRIVE_SPEED/1.5, 3, 0);
//
//        moveArm(0, -140, ARM_POWER/1.5);
//        int currElbowPos = Shawn.armElbow.getCurrentPosition();
//        moveArm(0, initElbowPos-currElbowPos, ARM_POWER/4);
////        for (int i = 0; i < 20; i++) {
////            moveArm(0, -10, ARM_POWER);
////      //      gyroHold(TURN_SPEED, 0, 0.3);
////            Thread.sleep(300);
////        }
//
//        gyroDrive(DRIVE_SPEED/1.5, -3.5, 0);



//        gyroDrive(DRIVE_SPEED, 6, 0.0);   // returns to original position
//
//        gyroTurn(TURN_SPEED, 90);           // turn 90 degrees to the right
//        gyroHold(TURN_SPEED, 90, 0.5);
//        gyroDrive(DRIVE_SPEED, -36, 90);     //drives to the safe zone
//        gyroHold(TURN_SPEED, 90, 0.5);
//        gyroTurn(TURN_SPEED, 0);
//        gyroHold(TURN_SPEED, 0, 0.5);

        Shawn.armShoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        moveArm(40, 0, 10, ARM_POWER/2);
        moveArm(-40,0, 20, ARM_POWER/2);
        Shawn.armShoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Shawn.armShoulder.setPower(0);
        Thread.sleep(1000);
        Shawn.armShoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        Thread.sleep(500);

        Shawn.armElbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Shawn.armElbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        moveArm(0, 100, 100, ARM_POWER/3);
        moveArm(0, 100, 5, ARM_POWER/4);
        Thread.sleep(10000);

//        gyroDrive(DRIVE_SPEED, -30, 0);
//        gyroTurn(TURN_SPEED, -45);
//        gyroHold(TURN_SPEED, -45, 0.5);
//        gyroDrive(DRIVE_SPEED, 40 , 45);
//        gyroTurn(TURN_SPEED, 0);
//        gyroHold(TURN_SPEED, 0, 0.5);

        telemetry.addData("Path", "Complete");
        telemetry.update();

    }


   /**
    *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
    *  Move will stop if either of these conditions occur:
    *  1) Move gets to the desired position
    *  2) Driver stops the opmode running.
    *
    * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
    * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
    * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
    *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
    *                   If a relative angle is required, add/subtract from current heading.
    */
    public void gyroDrive ( double speed, double distance, double angle) {

        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftTarget = Shawn.leftDrive.getCurrentPosition() + moveCounts;
            newRightTarget = Shawn.rightDrive.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            Shawn.leftDrive.setTargetPosition(newLeftTarget);
            Shawn.rightDrive.setTargetPosition(newRightTarget);

            Shawn.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Shawn.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(abs(speed), 0.0, 1.0);
            Shawn.leftDrive.setPower(speed);
            Shawn.rightDrive.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                   (Shawn.leftDrive.isBusy() && Shawn.rightDrive.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(abs(leftSpeed), abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                Shawn.leftDrive.setPower(leftSpeed);
                Shawn.rightDrive.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                telemetry.addData("Actual",  "%7d:%7d",      Shawn.leftDrive.getCurrentPosition(),
                                                             Shawn.rightDrive.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            Shawn.leftDrive.setPower(0);
            Shawn.rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            Shawn.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Shawn.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        Shawn.leftDrive.setPower(0);
        Shawn.rightDrive.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        Shawn.leftDrive.setPower(leftSpeed);
        Shawn.rightDrive.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the Shawn's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the Shawn's frame of reference
     *          +ve error means the Shawn should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        Orientation angles = Shawn.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        robotError = targetAngle - angles.firstAngle;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in Shawn relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public void moveArm(int shoulderDegrees, int  elbowDegrees, int increment, double power) {

        int currentShoulderPosition = Shawn.armShoulder.getCurrentPosition();
        int currentElbowPosition = Shawn.armElbow.getCurrentPosition();
        int finalShoulderPosition =  currentShoulderPosition+ (shoulderDegrees * TICKS_PER_DEGREE);
        int finalElbowPosition = currentElbowPosition + (elbowDegrees * TICKS_PER_DEGREE);

        if (shoulderDegrees > 0) {
            Shawn.armShoulder.setPower(power);
            while (currentShoulderPosition <= finalShoulderPosition) {
                Shawn.armShoulder.setTargetPosition(currentShoulderPosition + (abs(increment) * TICKS_PER_DEGREE));
                while (Shawn.armShoulder.isBusy()) {
                }
                currentShoulderPosition = Shawn.armShoulder.getCurrentPosition();
            }
        } else if (shoulderDegrees < 0) {
            Shawn.armShoulder.setPower(power);
            while (currentShoulderPosition >= finalShoulderPosition) {
                Shawn.armShoulder.setTargetPosition(currentShoulderPosition - (abs(increment) * TICKS_PER_DEGREE));
                while (Shawn.armShoulder.isBusy()) {
                }
                currentShoulderPosition = Shawn.armShoulder.getCurrentPosition();
            }
        }
        Shawn.armShoulder.setPower(0);

        if (elbowDegrees > 0) {
            Shawn.armElbow.setPower(power);
            while (currentElbowPosition <= finalElbowPosition) {
                Shawn.armElbow.setTargetPosition(currentElbowPosition + (abs(increment) * TICKS_PER_DEGREE));
                while (Shawn.armElbow.isBusy()) {
                }
                currentElbowPosition = Shawn.armElbow.getCurrentPosition();
            }
        } else if (elbowDegrees < 0) {
            Shawn.armElbow.setPower(power);
            while (currentElbowPosition >= finalElbowPosition) {
                Shawn.armElbow.setTargetPosition(currentElbowPosition - (abs(increment) * TICKS_PER_DEGREE));
                while (Shawn.armElbow.isBusy()) {
                }
                currentElbowPosition = Shawn.armElbow.getCurrentPosition();
            }
        }
        Shawn.armElbow.setPower(0);

    }

}
