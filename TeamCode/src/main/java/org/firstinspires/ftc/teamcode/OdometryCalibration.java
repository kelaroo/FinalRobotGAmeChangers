package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

@Autonomous(name = "Odometry Calibration")
public class OdometryCalibration extends LinearOpMode {

    static final double calibrationSpeed = 0.5;

    static final double TICKS_PER_REV = 383.6;
    static final double WHEEL_DIAMETER = 38 / 100; // 38 mm
    static final double GEAR_RATIO = 1;

    static final double TICKS_PER_CM = WHEEL_DIAMETER * Math.PI * GEAR_RATIO / TICKS_PER_REV;

    File sideWheelSeparationFile = AppUtil.getInstance().getSettingsFile("sideWheelSeparationFile");
    File middleTickOffsetFile = AppUtil.getInstance().getSettingsFile("middleTickOffsetFile");

    public HardwareConfig hw;

    ElapsedTime timer;

    DcMotor odoLeft;
    DcMotor odoRight;
    DcMotor odoCenter;

    @Override
    public void runOpMode() throws InterruptedException {
        timer = new ElapsedTime();
        odoCenter = hardwareMap.get(DcMotor.class, "right_back");
        odoLeft = hardwareMap.get(DcMotor.class, "right_front");
        odoRight = hardwareMap.get(DcMotor.class, "left_back");
        hw = new HardwareConfig(hardwareMap);

        resetOdoEncoders();

        /*odoCenter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        odoLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        odoRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/

        telemetry.addData("Status", "Ready");
        telemetry.update();



        waitForStart();

        while(hw.imu.getAngularOrientation().firstAngle < 90 && opModeIsActive()) {
            telemetry.addData("angle", hw.imu.getAngularOrientation().firstAngle);
            telemetry.update();
            double coeff = (hw.imu.getAngularOrientation().firstAngle < 60)? 1: 0.5;

            hw.leftBack.setPower(-calibrationSpeed * coeff);
            hw.leftFront.setPower(-calibrationSpeed * coeff);
            hw.rightBack.setPower(calibrationSpeed * coeff);
            hw.rightFront.setPower(calibrationSpeed * coeff);
        }
        for(DcMotor motor: hw.lDriveMotors) {
            motor.setPower(0);
        }

        timer.reset();
        while(timer.seconds() < 1 && opModeIsActive())
            ;

        double angle = hw.imu.getAngularOrientation().firstAngle;
        double encoderDiff = Math.abs(Math.abs(odoLeft.getCurrentPosition()) - Math.abs(odoRight.getCurrentPosition()));
        double sideEncoderTickOffset = encoderDiff / angle;
        double sideWheelSeparation = (180 * sideEncoderTickOffset) / (TICKS_PER_CM * Math.PI);
        double middleTickOffset = odoCenter.getCurrentPosition() / Math.toRadians(angle);

        ReadWriteFile.writeFile(sideWheelSeparationFile, String.valueOf(sideWheelSeparation));
        ReadWriteFile.writeFile(middleTickOffsetFile, String.valueOf(middleTickOffset));

        while(opModeIsActive()) {
            telemetry.addData("sideWheelSeparation", sideWheelSeparation);
            telemetry.addData("middleTickOffset", middleTickOffset);
            telemetry.update();
        }
    }

    void resetOdoEncoders() {
        odoLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoCenter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        odoLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odoRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odoCenter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
