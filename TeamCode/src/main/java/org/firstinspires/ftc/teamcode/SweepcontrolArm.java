package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

@TeleOp(name = "SweepControlArm", group = "Motors")
//@Disabled

public class SweepcontrolArm extends OpMode {

    HardwareShawn Shawn = new HardwareShawn();

    @Override
    public void init() {

        Shawn.init(hardwareMap, false);

        Shawn.ArmRotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Shawn.SprocketRotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Shawn.ArmRotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Shawn.SprocketRotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Shawn.ArmRotation.setPower(0);
        Shawn.SprocketRotation.setPower(0);
    }


    @Override
    public void loop() {

        int ArmCurrentPosition;
        int SprocketCurrentPosition;

        if (gamepad1.x) {
            ArmCurrentPosition = Shawn.ArmRotation.getCurrentPosition();
            Shawn.ArmRotation.setTargetPosition(ArmCurrentPosition + (1440*4));
            SprocketCurrentPosition = Shawn.SprocketRotation.getCurrentPosition();
            Shawn.SprocketRotation.setTargetPosition(SprocketCurrentPosition + (1440 * 4));
            Shawn.SprocketRotation.setPower(1);
            Shawn.ArmRotation.setPower(1);
            telemetry.addLine("X Pressed");
        } else if (gamepad1.b) {
            ArmCurrentPosition = Shawn.ArmRotation.getCurrentPosition();
            Shawn.ArmRotation.setTargetPosition(ArmCurrentPosition - (1440*4));
            SprocketCurrentPosition = Shawn.SprocketRotation.getCurrentPosition();
            Shawn.SprocketRotation.setTargetPosition(SprocketCurrentPosition - (1440 * 4));
            Shawn.SprocketRotation.setPower(1);
            Shawn.ArmRotation.setPower(1);
        }
        else    {
         Shawn.ArmRotation.setPower(0);
         Shawn.SprocketRotation.setPower(0);

        }


        telemetry.update();
        while (Shawn.ArmRotation.isBusy() && Shawn.SprocketRotation.isBusy()) {
        }
        Shawn.ArmRotation.setPower(0);
        Shawn.SprocketRotation.setPower(0);



    }

}

