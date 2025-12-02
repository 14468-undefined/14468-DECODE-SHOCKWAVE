package org.firstinspires.ftc.teamcode.auto.archive;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.subsystem.BaseRobot;

import org.firstinspires.ftc.teamcode.util.SampleAuto;

//@Autonomous(name="BOTREDAUTO")
public class BOTREDAUTO extends SampleAuto {
    private BaseRobot robot;


    private int shooterRPMClose = 2135;

    TelemetryPacket packet = new TelemetryPacket();

    @Override
    public void onInit() {
        robot = new BaseRobot(hardwareMap, new Pose2d(-61, 40, Math.toRadians(180)));

        robot.shooter.setTargetRPM(shooterRPMClose);
        //set pos of hood and transfer servo
        robot.intake.setIntakePower(.8);

        packet.put("target_shooter_rpm", robot.shooter.getTargetRPM());
        packet.put("current_shooter_rpm", robot.shooter.getShooterVelocity());


        robot.LED.startOscillating();
    }

    @Override
    public void onStart() {




        while (opModeIsActive()) {

            new Thread(() -> {
                double min = 0.28;
                double max = 0.72;
                double mid = (min + max) / 2.0;
                double range = (max - min) / 2.0;

                while (opModeIsActive()) {
                    double time = getRuntime();   // safe to use inside thread

                    // 6-second breathing cycle
                    double osc = mid + range * Math.sin(time * (2.0 * Math.PI / 6.0));

                    robot.LED.setPoseTest(osc);

                    // update ~25 times per second
                    try { Thread.sleep(40); } catch (InterruptedException e) {}
                }
            }).start();


            Actions.runBlocking(robot.drive.actionBuilder(robot.drive.getPose())
                    .waitSeconds(1)
                    .strafeToSplineHeading(new Vector2d(-32,32),Math.toRadians(135))//go shoot
                    .waitSeconds(4)
                    //go to pile
                    .strafeToSplineHeading(new Vector2d(-11.2, 25.4), Math.toRadians(90))
                    //pick up
                    .strafeToConstantHeading(new Vector2d(-11.2, 54.5))

                    //gate dump
                    .strafeToConstantHeading(new Vector2d(-11.2, 48))
                    .strafeToSplineHeading(new Vector2d(1.4, 55), Math.toRadians(90))//gate dump
                    .strafeToConstantHeading(new Vector2d(-17, 48))
                    //return\
                    .strafeToSplineHeading(new Vector2d(-32,32),Math.toRadians(135))//shoot

                    .waitSeconds(4)
                    .strafeToSplineHeading(new Vector2d(35.8, 29), Math.toRadians(90))//go to motif 1
                    .strafeToConstantHeading(new Vector2d(35.8, 61))//intake
                    .strafeToConstantHeading(new Vector2d(35.8, 29))//intake
                    .strafeToSplineHeading(new Vector2d(-32,32),Math.toRadians(135))//go shoot

                    .build());



            break;
        }
    }

    @Override
    public void onStop() {
        robot.stopAll();
    }
}