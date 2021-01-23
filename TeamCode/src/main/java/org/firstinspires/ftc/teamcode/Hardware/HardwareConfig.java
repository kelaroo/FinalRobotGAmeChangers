package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;
import java.util.List;
import java.util.zip.ZipError;

public class HardwareConfig {
    public DcMotor leftBack;
    public DcMotor rightBack;
    public DcMotor leftFront;
    public DcMotor rightFront;
    public List<DcMotor> lDriveMotors;

    public Servo apucat;
    public DcMotor brat;

    public BNO055IMU imu;

    public HardwareConfig(HardwareMap hw) {
        leftBack = hw.get(DcMotor.class, "left_back");
        rightBack = hw.get(DcMotor.class, "right_back");
        leftFront = hw.get(DcMotor.class, "left_front");
        rightFront = hw.get(DcMotor.class, "right_front");
        lDriveMotors = Arrays.asList(leftFront, leftBack, rightFront, rightBack);

        for(DcMotor motor: lDriveMotors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        brat = hw.get(DcMotor.class, "odoRight");
        brat.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        apucat = hw.get(Servo.class, "apucat");
        apucat.setPosition(0);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        imu = hw.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public double clipPower(double power) {
        return (power > 1)? 1: ((power < -1)? -1: power);
    }
}
