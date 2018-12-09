package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "SweepControlArm", group = "Motors")
//@Disabled

public class SweepcontrolArm extends OpMode {

    HardwareShawn Shawn = new HardwareShawn();

    @Override
    public void init() {

        Shawn.init(hardwareMap, false);

        Shawn.armRotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Shawn.harvester.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Shawn.armRotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Shawn.harvester.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Shawn.armRotation.setPower(0);
        Shawn.harvester.setPower(0);
    }


    @Override
    public void loop() {

        int ArmCurrentPosition;
        int SprocketCurrentPosition;

        if (gamepad1.x) {
            ArmCurrentPosition = Shawn.armRotation.getCurrentPosition();
            Shawn.armRotation.setTargetPosition(ArmCurrentPosition + (1440*4));
            SprocketCurrentPosition = Shawn.harvester.getCurrentPosition();
            Shawn.harvester.setTargetPosition(SprocketCurrentPosition + (1440 * 4));
            Shawn.harvester.setPower(1);
            Shawn.armRotation.setPower(1);
            telemetry.addLine("X Pressed");
        } else if (gamepad1.b) {
            ArmCurrentPosition = Shawn.armRotation.getCurrentPosition();
            Shawn.armRotation.setTargetPosition(ArmCurrentPosition - (1440*4));
            SprocketCurrentPosition = Shawn.harvester.getCurrentPosition();
            Shawn.harvester.setTargetPosition(SprocketCurrentPosition - (1440 * 4));
            Shawn.harvester.setPower(1);
            Shawn.armRotation.setPower(1);
        }
        else    {
         Shawn.armRotation.setPower(0);
         Shawn.harvester.setPower(0);

        }


        telemetry.update();
        while (Shawn.armRotation.isBusy() && Shawn.harvester.isBusy()) {
        }
        Shawn.armRotation.setPower(0);
        Shawn.harvester.setPower(0);



    }

}

