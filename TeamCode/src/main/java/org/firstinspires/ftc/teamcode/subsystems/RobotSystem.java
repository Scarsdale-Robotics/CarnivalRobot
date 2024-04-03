package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareRobot;

public class RobotSystem {
    private HardwareRobot hardwareRobot;
    private InDepSubsystem inDep;
    private DriveSubsystem drive;

    public RobotSystem(HardwareMap hardwareMap, LinearOpMode opMode, Telemetry telemetry) {
        this.hardwareRobot = new HardwareRobot(hardwareMap);
        inDep = new InDepSubsystem(
                hardwareRobot.leftArm,
                hardwareRobot.rightArm,
                hardwareRobot.elbow,
                hardwareRobot.wrist,
                hardwareRobot.leftClaw,
                hardwareRobot.rightClaw,
                opMode,
                telemetry
        );
        drive = new DriveSubsystem(
                hardwareRobot.leftFront,
                hardwareRobot.rightFront,
                hardwareRobot.leftBack,
                hardwareRobot.rightBack,
                hardwareRobot.imu,
                opMode,
                telemetry
        );
        drive.resetIMU();
    }

    public HardwareRobot getHardwareRobot() {
        return hardwareRobot;
    }
    public AdafruitBNO055IMU getIMU() {
        return hardwareRobot.imu;
    }
    public InDepSubsystem getInDep() {
        return inDep;
    }
    public DriveSubsystem getDrive() {
        return drive;
    }
}