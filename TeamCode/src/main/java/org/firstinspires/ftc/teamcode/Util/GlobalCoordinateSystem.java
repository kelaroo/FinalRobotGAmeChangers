package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.teamcode.Hardware.OdometryConfig;

public class GlobalCoordinateSystem implements Runnable {

    OdometryConfig odometry;

    boolean isRunning = true;

    double leftEncoderPosition, rightEncoderPosition, middleEncoderPosition;
    double changeInOrientation;
    double OLDLeftEncoderPosition, OLDRightEncoderPosition, OLDMiddleEncoderPosition;

    double globalX, globalY, robotOrientation;

    double encoderWheelDistance;
    double middleEncoderTickOffset;

    int sleepTime;

    public GlobalCoordinateSystem(OdometryConfig odo, int threadSleepDelay) {
        odometry = odo;
        sleepTime = threadSleepDelay;

        encoderWheelDistance = Double.parseDouble(ReadWriteFile.readFile(odo.sideWheelSeparationFile).trim());
        middleEncoderTickOffset = Double.parseDouble(ReadWriteFile.readFile(odo.middleTickOffsetFile).trim());
    }

    public void positionUpdate() {
        leftEncoderPosition = odometry.odoLeft.getCurrentPosition();
        rightEncoderPosition = odometry.odoRight.getCurrentPosition();
        middleEncoderPosition = odometry.odoCenter.getCurrentPosition();

        double leftChange = leftEncoderPosition - OLDLeftEncoderPosition;
        double rightChange = rightEncoderPosition - OLDRightEncoderPosition;

        changeInOrientation = (leftChange-rightChange) / (encoderWheelDistance * odometry.TICKS_PER_CM);
        robotOrientation += changeInOrientation;

        double rawHorizontalChange = middleEncoderPosition - OLDMiddleEncoderPosition;
        double horizontalChange = rawHorizontalChange - (changeInOrientation * middleEncoderTickOffset);

        double sides = (rightChange + leftChange) / 2;
        double frontBack = horizontalChange;

        globalX += sides * Math.sin(robotOrientation) + frontBack * Math.cos(robotOrientation);
        globalY += sides * Math.cos(robotOrientation) - frontBack * Math.sin(robotOrientation);

        OLDLeftEncoderPosition = leftEncoderPosition;
        OLDRightEncoderPosition = rightEncoderPosition;
        OLDMiddleEncoderPosition = middleEncoderPosition;
    }

    public double getX() {return globalX;}
    public double getY() {return globalY;}
    public double getOrientation() {return Math.toDegrees(robotOrientation) % 360;}

    public void stop() {isRunning = false;}


    @Override
    public void run() {
        while(isRunning) {
            positionUpdate();
            try {
                Thread.sleep(sleepTime);
            }catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
