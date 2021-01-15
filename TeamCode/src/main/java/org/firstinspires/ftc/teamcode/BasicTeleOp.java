package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name = "BasicTeleOp")
public class BasicTeleOp extends OpMode {
    private HardwareConfig hw;
    private final double COEFF_DRIVE_HIGH = 0.7;
    private final double COEFF_DRIVE_LOW = 0.4;

    private double coeff = COEFF_DRIVE_HIGH;

    @Override
    public void init() {
        hw = new HardwareConfig(hardwareMap);
    }

    @Override
    public void loop() {
        telemetry.addData("Angle1", hw.imu.getAngularOrientation().firstAngle);
        telemetry.addData("Angle2", hw.imu.getAngularOrientation().secondAngle);
        telemetry.addData("Angle3", hw.imu.getAngularOrientation().thirdAngle);
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        if(gamepad1.x)
            coeff = COEFF_DRIVE_HIGH;
        if(gamepad1.b)
            coeff = COEFF_DRIVE_LOW;

        if(coeff == COEFF_DRIVE_HIGH)
            telemetry.addData("Speed", "high");
        else
            telemetry.addData("Speed", "low");


        double LF = hw.clipPower(drive + strafe + rotate) * coeff;
        double LB = hw.clipPower(drive - strafe + rotate) * coeff;
        double RF = hw.clipPower(drive - strafe - rotate) * coeff;
        double RB = hw.clipPower(drive + strafe - rotate) * coeff;

        telemetry.addData("LF", LF);
        telemetry.addData("LB", LB);
        telemetry.addData("RF", RF);
        telemetry.addData("RB", RB);

        hw.leftFront.setPower(LF);
        hw.leftBack.setPower(LB);
        hw.rightFront.setPower(RF);
        hw.rightBack.setPower(RB);
    }
}
