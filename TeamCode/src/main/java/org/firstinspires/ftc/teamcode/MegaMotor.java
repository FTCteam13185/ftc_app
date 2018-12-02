package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.HardwareMap;

public class MegaMotor {

    public DcMotor myMotor = null;

    private int numTics;

    public MegaMotor(String motorName, int ticsPerRot) {
        HardwareMap hwMap = null;

        myMotor = hwMap.get(DcMotor.class, motorName);
        numTics = ticsPerRot;
    }

}
