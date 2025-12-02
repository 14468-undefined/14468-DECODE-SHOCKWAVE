package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystem.BaseRobot;

import org.firstinspires.ftc.teamcode.util.SampleAuto;

@Autonomous(name="ParkRedFar")
public class D_ParkRedFar extends SampleAuto {
    private BaseRobot robot;

    private int shooterRPMClose = 2135;

    TelemetryPacket packet = new TelemetryPacket();

    @Override
    public void onInit() {
        robot = new BaseRobot(hardwareMap, new Pose2d(61, 18, Math.toRadians(180)));

        robot.shooter.setTargetRPM(shooterRPMClose);



        packet.put("target_shooter_rpm", robot.shooter.getTargetRPM());
        packet.put("current_shooter_rpm", robot.shooter.getShooterVelocity());


        robot.LED.startOscillating();

    }

    @Override
    public void onStart() {

        //PARK
        Actions.runBlocking(robot.drive.actionBuilder(robot.drive.getPose())
                .strafeToConstantHeading(new Vector2d(60,30))//go park off to the side
                .build());




    }

    @Override
    public void onStop() {
        robot.stopAll();
    }
}