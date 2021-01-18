package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.HardwareConfig;
import org.firstinspires.ftc.teamcode.Hardware.OdometryConfig;
import org.firstinspires.ftc.teamcode.Util.GlobalCoordinateSystem;

@TeleOp
public class LocalizationTest extends LinearOpMode {

    HardwareConfig hw;
    OdometryConfig odometry;

    GlobalCoordinateSystem globalSpace;

    private final double COEFF_DRIVE_HIGH = 0.7;
    private final double COEFF_DRIVE_LOW = 0.4;

    private double coeff = COEFF_DRIVE_HIGH;

    @Override
    public void runOpMode() throws InterruptedException {
        hw = new HardwareConfig(hardwareMap);
        odometry = new OdometryConfig(hardwareMap);

        odometry.resetOdometryEncoders();

        waitForStart();

        globalSpace = new GlobalCoordinateSystem(odometry, 100);
        Thread positionUpdate = new Thread(globalSpace);
        positionUpdate.start();

        while(opModeIsActive()) {
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

            telemetry.addData("X", globalSpace.getX() / odometry.TICKS_PER_CM);
            telemetry.addData("Y", globalSpace.getY() / odometry.TICKS_PER_CM);
            telemetry.addData("odoLeft", odometry.odoLeft.getCurrentPosition());
            telemetry.addData("odoCenter", odometry.odoCenter.getCurrentPosition());
            telemetry.addData("odoRight", odometry.odoRight.getCurrentPosition());
            telemetry.addData("distanceLeft", odometry.odoLeft.getCurrentPosition() / odometry.TICKS_PER_CM);
            telemetry.addData("orientation(deg)", globalSpace.getOrientation());
            telemetry.update();
        }

        globalSpace.stop();
    }
}
