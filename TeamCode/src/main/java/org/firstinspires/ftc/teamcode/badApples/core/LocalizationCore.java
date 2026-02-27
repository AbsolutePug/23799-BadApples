package org.firstinspires.ftc.teamcode.badApples.core;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

public class LocalizationCore extends RobotFunctionCore {
    // Hardware
    private SparkFunOTOS otos;
    private IMU imu;

    // Variables
    public double IMU_YAW = 0;

    // Init localization
    public void initGyro(HardwareMap hwMap) {
        imu = hwMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot robot_orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        );

        imu.initialize(new IMU.Parameters(robot_orientation));
    }
    public void initOTOS(HardwareMap hwMap) {
        otos = hwMap.get(SparkFunOTOS.class, "sensor_otos");
    }

    // Gyro
    public double getHeading() {
        return IMU_YAW;
    }
    public void resetHeading() {
        imu.resetYaw();
    }
    public void updateGyro() {
        IMU_YAW = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
}
