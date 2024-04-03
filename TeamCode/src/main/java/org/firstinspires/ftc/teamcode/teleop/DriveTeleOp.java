package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.InDepSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotSystem;

@TeleOp(name = "Drive TeleOp")
public class DriveTeleOp extends LinearOpMode {
    private RobotSystem robot;
    private DriveSubsystem drive;
    private InDepSubsystem inDep;
    private final double drive_speed = 0.5;
    private final double arm_speed = 0.25;


    @Override
    public void runOpMode() {
        // init robot
        robot = new RobotSystem(hardwareMap, this, telemetry);
        drive = robot.getDrive();
        inDep = robot.getInDep();

        waitForStart();

        // loop
        while (opModeIsActive()) {
            ////////////////////
            // DRIVE CONTROLS //
            ////////////////////
            double x = 0, y = 0, turn = 0;
            if (Math.abs(gamepad1.left_stick_x) > 0.05)
                x = -gamepad1.left_stick_x;
            if (Math.abs(gamepad1.left_stick_y) > 0.05)
                y = gamepad1.left_stick_y;
            if (Math.abs(gamepad1.right_stick_x) > 0.05)
                turn = -gamepad1.right_stick_x;
            drive.driveFieldCentric(drive_speed * x, drive_speed * y, drive_speed * turn);

            ////////////////////
            // INDEP CONTROLS //
            ////////////////////
            double total = gamepad1.right_trigger - gamepad1.left_trigger;
            inDep.rawPower(arm_speed * total);

        }
        drive.stopController();
        inDep.stopMotors();
    }
}