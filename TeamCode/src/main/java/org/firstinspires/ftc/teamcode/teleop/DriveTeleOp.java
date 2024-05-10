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
    private final double drive_speed = 0.65;
    private final double arm_speed = 0.75;


    @Override
    public void runOpMode() {
        // init robot
        robot = new RobotSystem(hardwareMap, this, telemetry);
        drive = robot.getDrive();
        inDep = robot.getInDep();

        waitForStart();


        // loop
        boolean toggleLeftClaw = false, toggleRightClaw = false, toggleBothClaws = false;
        boolean leftClawOpen = false, rightClawOpen = false;
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
                turn = gamepad1.right_stick_x;
            drive.driveFieldCentric(drive_speed * x, drive_speed * y, drive_speed * turn);

            ////////////////////
            // INDEP CONTROLS //
            ////////////////////
            double totalPower = gamepad1.right_trigger - gamepad1.left_trigger;
            inDep.rawPower(arm_speed * totalPower);

            // triangle = both, square = left, circle = right
            if (gamepad1.triangle && !toggleBothClaws) {
                if (leftClawOpen || rightClawOpen) {
                    inDep.close();
                    leftClawOpen = false;
                    rightClawOpen = false;
                } else {
                    inDep.open();
                    leftClawOpen = true;
                    rightClawOpen = true;
                }
                toggleBothClaws = true;
            }
            if (!gamepad1.triangle) toggleBothClaws = false;

            if (gamepad1.square && !toggleLeftClaw) {
                if (leftClawOpen) {
                    inDep.closeLeft();
                    leftClawOpen = false;
                } else {
                    inDep.openLeft();
                    leftClawOpen = true;
                }
                toggleLeftClaw = true;
            }
            if (!gamepad1.square) toggleLeftClaw = false;

            if (gamepad1.circle && !toggleRightClaw) {
                if (rightClawOpen) {
                    inDep.closeRight();
                    rightClawOpen = false;
                } else {
                    inDep.openRight();
                    rightClawOpen = true;
                }
                toggleRightClaw = true;
            }
            if (!gamepad1.circle) toggleRightClaw = false;


        }
        drive.stopController();
        inDep.stopMotors();
    }
}