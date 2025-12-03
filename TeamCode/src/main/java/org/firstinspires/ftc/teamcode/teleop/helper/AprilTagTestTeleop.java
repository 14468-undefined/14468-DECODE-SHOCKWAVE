package org.firstinspires.ftc.teamcode.teleop.helper;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.BaseRobot;

@TeleOp(name = "AprilTags", group = "helper")
public class AprilTagTestTeleop extends LinearOpMode {

    BaseRobot robot;
    CommandBase driveToTagCommand;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize robot
        robot = new BaseRobot(hardwareMap, new Pose2d(0, 0, 0));

        // --- START CAMERA STREAM ---
        if (robot.webcamVision != null && robot.webcamVision.webcam != null) {
            robot.webcamVision.webcam.resumeStreaming();
            robot.webcamVision.webcam.resumeLiveView();


        }

        waitForStart();

        // Safety check: make sure drive subsystem exists
        if (robot.drive == null || robot.drive.drive == null) {
            telemetry.addLine("Drive subsystem not ready!");
            telemetry.update();
            return;
        }

        // Schedule the drive-to-tag command safely
        driveToTagCommand = robot.webcamVision.getDriveToTagCommand(robot.drive);
        driveToTagCommand.schedule();

        while (opModeIsActive()) {

            // Run FTCLib scheduler so commands actually execute
            CommandScheduler.getInstance().run();

            // Manual control when no tag detected
            if (!robot.webcamVision.hasTarget()) {
                double forward = -gamepad1.left_stick_y;
                double strafe = gamepad1.left_stick_x;
                double turn = -gamepad1.right_stick_x;

                if (robot.drive != null && robot.drive.drive != null) {
                    robot.drive.driveFieldcentric(strafe, forward, turn, 1.0);
                }
            }

            // Telemetry
            if (robot.cTelemetry != null) {
                robot.cTelemetry.reset();
                robot.printTelemetry(robot.cTelemetry);
                robot.cTelemetry.update();
            }
        }

        // Cancel command on exit
        if (driveToTagCommand != null) driveToTagCommand.cancel();
        CommandScheduler.getInstance().reset();
    }
}
