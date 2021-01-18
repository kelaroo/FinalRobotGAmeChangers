package org.firstinspires.ftc.teamcode.Autonomous;

import android.os.storage.StorageManager;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Hardware.HardwareConfig;
import org.firstinspires.ftc.teamcode.Hardware.OdometryConfig;

import java.io.File;

@Autonomous(name = "Odometry Calibration")
public class OdometryCalibration extends LinearOpMode {

    static final double calibrationSpeed = 0.5;

    HardwareConfig hw;
    OdometryConfig odometry;

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        hw = new HardwareConfig(hardwareMap);
        odometry = new OdometryConfig(hardwareMap);

        odometry.resetOdometryEncoders();

        waitForStart();

        while(hw.imu.getAngularOrientation().firstAngle < 90 && opModeIsActive()) {
            double power = calibrationSpeed * ((hw.imu.getAngularOrientation().firstAngle < 60)? 1: 0.5);
            hw.rightFront.setPower(power);
            hw.rightBack.setPower(power);
            hw.leftBack.setPower(-power);
            hw.leftFront.setPower(-power);
        }

        hw.rightFront.setPower(0);
        hw.rightBack.setPower(0);
        hw.leftBack.setPower(0);
        hw.leftFront.setPower(0);

        timer.reset();
        while(timer.seconds() < 1 && opModeIsActive())
            ;

        double angle = hw.imu.getAngularOrientation().firstAngle;
        double encoderDifference = Math.abs(odometry.odoLeft.getCurrentPosition() - odometry.odoRight.getCurrentPosition());

        double sideWheelSeparation = (encoderDifference / odometry.TICKS_PER_CM) / Math.toRadians(angle);
        double sideEncoderTickOffset = encoderDifference / angle;
        //double sideWheelSeparation = (sideEncoderTickOffset*odometry.TICKS_PER_CM) / (Math.PI / 180);  //(180 * sideEncoderTickOffset) / (odometry.TICKS_PER_CM * Math.PI);
        double middleTickOffset = odometry.odoCenter.getCurrentPosition() / Math.toRadians(angle);

        ReadWriteFile.writeFile(odometry.sideWheelSeparationFile, String.valueOf(sideWheelSeparation));
        ReadWriteFile.writeFile(odometry.middleTickOffsetFile, String.valueOf(middleTickOffset));

        while(opModeIsActive()) {
            telemetry.addData("sideWheelSeparation", sideWheelSeparation);
            telemetry.addData("angle", angle);
            telemetry.addData("ticksPerCm", odometry.TICKS_PER_CM);
            telemetry.addData("encoderDifference", encoderDifference);
            telemetry.addData("middleTickOffset", middleTickOffset);

            telemetry.addData("odoLeft", odometry.odoLeft.getCurrentPosition());
            telemetry.addData("odoCenter", odometry.odoCenter.getCurrentPosition());
            telemetry.addData("odoRight", odometry.odoRight.getCurrentPosition());
            telemetry.update();
        }
    }
}
