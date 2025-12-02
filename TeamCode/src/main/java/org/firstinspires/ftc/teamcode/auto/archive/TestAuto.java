package org.firstinspires.ftc.teamcode.auto.archive;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystem.BaseRobot;

import org.firstinspires.ftc.teamcode.util.SampleAuto;

@Autonomous(name="TEST")
public class TestAuto extends SampleAuto {
    private BaseRobot robot;



    @Override
    public void onInit() {
        robot = new BaseRobot(hardwareMap, new Pose2d(-61, 40, Math.toRadians(180)));



        robot.LED.startOscillating();

    }

    @Override
    public void onStart() {

        robot.delay(1);
        robot.LED.startOscillating();

        robot.delay(28);
    }

    @Override
    public void onStop() {

    }
}