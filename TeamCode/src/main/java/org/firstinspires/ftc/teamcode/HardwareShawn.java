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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

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

    public BNO055IMU imu = null;
    public DcMotor armRotation = null;
    public DcMotor harvester = null;


    public DcMotor leftRear = null;
    public DcMotor rightRear = null;
    public DcMotor leftFront = null;
    public DcMotor rightFront = null;

    public DcMotor actuator = null;
 //   public DcMotor sweepy = null;
    public CRServo claw = null;

    public DigitalChannel touchSensor = null;
 //   public ColorSensor colorSensor = null;

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
        /*
        armRotation = hwMap.get(DcMotor.class, "Arm_Rotation" ) ;
        harvester = hwMap.get(DcMotor.class, "Sprocket_Rotation");
        harvester.setMode (DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        harvester.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        armRotation.setPower(0);
        harvester.setPower(0);
        armRotation.setDirection (DcMotor.Direction.FORWARD);
        harvester.setDirection (DcMotor.Direction.FORWARD);
        armRotation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        harvester.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
*/
        actuator = hwMap.get(DcMotor.class, "actuator");

    //    sweepy = hwMap.get(DcMotor.class, "sweepy");

        leftRear = hwMap.get(DcMotor.class, "leftRear");
        rightRear = hwMap.get(DcMotor.class, "rightRear");
        leftFront = hwMap.get(DcMotor.class, "leftFront");
        rightFront = hwMap.get(DcMotor.class, "rightFront");

 //       touchSensor = hwMap.get(DigitalChannel.class, "touchSensor");
 //       touchSensor.setMode(DigitalChannel.Mode.INPUT);

        actuator.setPower(0);
   //     sweepy.setPower(0);

        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftRear.setPower(0);
        rightRear.setPower(0);
        leftFront.setPower(0);
        rightFront.setPower(0);
//
//        imu = hwMap.get(BNO055IMU.class, "imu");
//
        actuator.setDirection(DcMotor.Direction.FORWARD);
   //     sweepy.setDirection(DcMotor.Direction.FORWARD);
 //       claw.setDirection(DcMotor.Direction.FORWARD);

        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);

//        // Set all motors to run without encoders.
//        // May want to use RUN_USING_ENCODERS if encoders are installed.
//
        //actuator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
     //   newMotor.myMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
     //   newMotor.myMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        actuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
   //     sweepy.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    //    rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        actuator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    //    sweepy.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }


 }
