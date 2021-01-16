package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

public class OdometryConfig {
    public DcMotor odoCenter;
    public DcMotor odoLeft;
    public DcMotor odoRight;

    public final double TICKS_PER_REV = 383.6;
    public final double WHEEL_DIAMETER = 38 / 100; // 38 mm
    public final double GEAR_RATIO = 1;

    public final double TICKS_PER_CM = WHEEL_DIAMETER * Math.PI * GEAR_RATIO / TICKS_PER_REV;

    public File sideWheelSeparationFile = AppUtil.getInstance().getSettingsFile("sideWheelSeparationFile");
    public File middleTickOffsetFile = AppUtil.getInstance().getSettingsFile("middleTickOffsetFile");

    public OdometryConfig(HardwareMap hw) {
        odoCenter = hw.get(DcMotor.class, "odoCenter");
        odoLeft = hw.get(DcMotor.class, "odoLeft");
        odoRight = hw.get(DcMotor.class, "odoRight");

        odoCenter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odoLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odoRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetOdometryEncoders() {
        odoCenter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        odoCenter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odoLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odoRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
