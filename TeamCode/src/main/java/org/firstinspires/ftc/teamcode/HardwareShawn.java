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
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single Shawn.ould
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the Shawn:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwareShawn
{
    /* Public OpMode members. */
    public DcMotor  leftDrive   = null;
    public DcMotor  rightDrive  = null;
    public DcMotor  armElbow    = null;
    public DcMotor armShoulder = null;
    public BNO055IMU imu = null;
    public DcMotor ArmRotation = null;
    public DcMotor SprocketRotation = null;
//    public CRServo armClaw = null;
    public Servo armServo = null;
    public Servo leftClaw = null;
    public Servo rightClaw = null;
    public Servo tailServo = null;
    public Servo tailEnd = null;

    public DcMotor leftRear = null;
    public DcMotor rightRear = null;
    public DcMotor leftFront = null;
    public DcMotor rightFront = null;

    public ColorSensor colorSensor = null;

//    public DcMotor  leftArm     = null;
//    public Servo    leftClaw    = null;
//    public Servo    rightClaw   = null;
//
//    public static final double MID_SERVO       =  0.5 ;
//    public static final double ARM_UP_POWER    =  0.45 ;
//    public static final double ARM_DOWN_POWER  = -0.45 ;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareShawn(){

}

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, boolean gyro) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        if (gyro) {
            // IMU Calibration
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            imu = hwMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);

            BNO055IMU.CalibrationData calibrationData = imu.readCalibrationData();

            // Save the calibration data to a file. You can choose whatever file
            // name you wish here, but you'll want to indicate the same file name
            // when you initialize the IMU in an opmode in which it is used. If you
            // have more than one IMU on your robot, you'll of course want to use
            // different configuration file names for each.
            String filename = "AdafruitIMUCalibration.json";
            File file = AppUtil.getInstance().getSettingsFile(filename);
            ReadWriteFile.writeFile(file, calibrationData.serialize());

            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            imu.initialize(parameters);
        }

        // Define and Initialize Motors
        ArmRotation = hwMap.get(DcMotor.class, "Arm_Rotation" ) ;
        SprocketRotation = hwMap.get(DcMotor.class, "Sprocket_Rotation");
        SprocketRotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ArmRotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
// leftDrive   = hwMap.get(DcMotor.class, "left_drive");
//        rightDrive  = hwMap.get(DcMotor.class, "right_drive");
//        armElbow    = hwMap.get(DcMotor.class, "arm_elbow");
//        armShoulder = hwMap.get(DcMotor.class, "arm_shoulder");
   //     armClaw     = hwMap.get(CRServo.class, "armClaw");
//        armServo    = hwMap.get(Servo.class, "armServo");
//        leftClaw    = hwMap.get(Servo.class, "leftClaw");
//        rightClaw   = hwMap.get(Servo.class, "rightClaw");
//        tailServo   = hwMap.get(Servo.class, "tailServo");
//        tailEnd     = hwMap.get(Servo.class, "tailEnd");

//        rightRear = hwMap.get(DcMotor.class, "rightRear");
//        leftFront = hwMap.get(DcMotor.class, "leftFront");
//        rightFront = hwMap.get(DcMotor.class, "rightFront");

  //      colorSensor = hwMap.get(ColorSensor.class, "sensor_color");

  //      imu = hwMap.get(BNO055IMU.class, "imu");

  //      rightRear.setDirection(DcMotorSimple.Direction.FORWARD);
  //      leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
  //      rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

  //      rightRear.setPower(0);
  //      leftFront.setPower(0);
  //      rightFront.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.

//        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }


 }
