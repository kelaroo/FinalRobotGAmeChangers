package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.HardwareConfig;

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
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        if(gamepad1.x)
            coeff = COEFF_DRIVE_HIGH;
        if(gamepad1.b)
            coeff = COEFF_DRIVE_LOW;

        double LF = hw.clipPower(drive + strafe + rotate) * coeff;
        double LB = hw.clipPower(drive - strafe + rotate) * coeff;
        double RF = hw.clipPower(drive - strafe - rotate) * coeff;
        double RB = hw.clipPower(drive + strafe - rotate) * coeff;

        hw.leftFront.setPower(LF);
        hw.leftBack.setPower(LB);
        hw.rightFront.setPower(RF);
        hw.rightBack.setPower(RB);

        double lTrigger = gamepad1.left_trigger;
        double rTrigger = gamepad1.right_trigger;
        if(lTrigger > 0.1 || rTrigger > 0.1) {
            hw.brat.setPower(rTrigger - lTrigger);
        } else {
            hw.brat.setPower(0);
        }

        if(gamepad1.left_bumper) {
            hw.apucat.setPosition(1);
        } else if(gamepad1.right_bumper) {
            hw.apucat.setPosition(0);
        }

    }
}
