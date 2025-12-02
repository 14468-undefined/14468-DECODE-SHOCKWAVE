package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.util.AutoUtil;
import org.firstinspires.ftc.teamcode.subsystem.BaseRobot;

import org.firstinspires.ftc.teamcode.util.SampleAuto;

@Autonomous(name="ONLY3RedNear")
public class C_ONLY3RedNear extends SampleAuto {
    private BaseRobot robot;

    private int shooterRPMClose = 2135;

    TelemetryPacket packet = new TelemetryPacket();

    @Override
    public void onInit() {
        robot = new BaseRobot(hardwareMap, new Pose2d(-61, 40, Math.toRadians(180)));

        robot.shooter.setTargetRPM(shooterRPMClose);

        robot.intake.setIntakePower(.8);

        packet.put("target_shooter_rpm", robot.shooter.getTargetRPM());
        packet.put("current_shooter_rpm", robot.shooter.getShooterVelocity());


        robot.LED.startOscillating();


    }

    @Override
    public void onStart() {

        Actions.runBlocking((t) -> {robot.shooter.spin(); return false; });
        Actions.runBlocking(robot.drive.actionBuilder(robot.drive.getPose())
                .strafeToSplineHeading(new Vector2d(-27,24),Math.toRadians(133))//go to shooting pose
                .build());



        Actions.runBlocking((t) -> {robot.intake.intake(); return false; });
        Actions.runBlocking((t) -> {robot.transfer.spin(); return false; });
        AutoUtil.delay(2.9);

        Actions.runBlocking((t) -> {robot.stopAll(); return false; });

        //PARK
        Actions.runBlocking(robot.drive.actionBuilder(robot.drive.getPose())


//park
                .strafeToSplineHeading(new Vector2d(-55, 24), Math.toRadians(180))
                .build());


    }

    @Override
    public void onStop() {
        robot.stopAll();
    }
}