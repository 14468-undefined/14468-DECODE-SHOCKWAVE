package org.firstinspires.ftc.teamcode.auto.archive;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.subsystem.BaseRobot;

import org.firstinspires.ftc.teamcode.util.SampleAuto;

//@Autonomous(name="BlueNearSuperSpeed")
public class BlueNearSuperSpeed extends SampleAuto {
    private BaseRobot robot;


    private int shooterRPMClose = 2135;

    TelemetryPacket packet = new TelemetryPacket();

    @Override
    public void onInit() {
        robot = new BaseRobot(hardwareMap, new Pose2d(-61, -40, Math.toRadians(180)));

        robot.shooter.setTargetRPM(shooterRPMClose);
        //set pos of hood and transfer servo
        robot.intake.setIntakePower(.8);

        packet.put("target_shooter_rpm", robot.shooter.getTargetRPM());
        packet.put("current_shooter_rpm", robot.shooter.getShooterVelocity());


    }

    @Override
    public void onStart() {

        robot.shooter.spin();
        Actions.runBlocking(robot.drive.actionBuilder(robot.drive.getPose())
                .strafeToSplineHeading(new Vector2d(-27,-24),Math.toRadians(227))//go to shooting pose
                .build());




        robot.intake.intake();
        robot.delay(2.9);
        //robot.shooter.spinUpChooseRPM(shooterRPMClose+55);
        //robot.delay(1);
        robot.intake.stop();
        robot.intake.setIntakePower(1);
        robot.shooter.eStop();
        // ====================== Intake 1st Pile ====================== \\
        boolean[] intakeStarted = {false};
        boolean[] intakeStopped = {false};
        boolean[] shooterReverseStart = {false};
        boolean[] shooterReverseStop = {false};

        robot.intake.setIntakePower(1);
        Actions.runBlocking(robot.drive.actionBuilder(robot.drive.getPose())

                .afterTime(0, t -> {
                    if (!opModeIsActive()) return false;
                    if (!intakeStarted[0]) {
                        robot.intake.intake();
                        intakeStarted[0] = true;
                    }
                    return !intakeStarted[0]; // becomes false after first trigger = DISABLE marker
                })

                .afterTime(3, t -> {//was 3.7
                    if (!opModeIsActive()) return false;
                    if (!intakeStopped[0]) {
                        robot.intake.stop();
                        intakeStopped[0] = true;
                    }
                    return !intakeStopped[0]; // disables marker after stop fires
                })

                .afterTime(2, t -> {
                    if (!opModeIsActive()) return false;
                    if (!shooterReverseStart[0]) {
                        robot.shooter.setTargetRPM(-1000);
                        robot.shooter.spin();
                        shooterReverseStart[0] = true;
                    }
                    return !shooterReverseStart[0]; // disables marker after stop fires
                })

                .afterTime(3.8, t -> {
                    if (!opModeIsActive()) return false;
                    if (!shooterReverseStop[0]) {
                        robot.shooter.eStop();
                        robot.shooter.setTargetRPM(shooterRPMClose);
                        shooterReverseStop[0] = true;
                    }
                    return !shooterReverseStop[0]; // disables marker after stop fires
                })


                .strafeToSplineHeading(new Vector2d(-3, -20), Math.toRadians(270), new TranslationalVelConstraint(100))
                .strafeToConstantHeading(new Vector2d(-3, -63), new TranslationalVelConstraint(30))


                .strafeToConstantHeading(new Vector2d(-3, -57), new TranslationalVelConstraint(100))
                .strafeToSplineHeading(new Vector2d(-28, -20), Math.toRadians(221), new TranslationalVelConstraint(100))
                .build());

        robot.shooter.spin();
        robot.delay(2);
        robot.intake.setIntakePower(1);
        robot.intake.intake();
        robot.delay(3);
        robot.intake.stop();
        robot.shooter.eStop();
        //wait til shoot 3 is done to move



        intakeStarted[0] = false;
        intakeStopped[0] = false;
        shooterReverseStart[0] = false;
        shooterReverseStop[0] = false;

        Actions.runBlocking(robot.drive.actionBuilder(robot.drive.getPose())

                .afterTime(1.6, t -> {
                    if (!opModeIsActive()) return false;
                    if (!intakeStarted[0]) {
                        robot.intake.intake();
                        intakeStarted[0] = true;
                    }
                    return !intakeStarted[0]; // becomes false after first trigger
                })

                .afterTime(3.87 , t -> {
                    if (!opModeIsActive()) return false;
                    if (!intakeStopped[0]) {
                        robot.intake.stop();
                        intakeStopped[0] = true;
                    }
                    return !intakeStopped[0];
                })

                .afterTime(2, t -> {
                    if (!opModeIsActive()) return false;
                    if (!shooterReverseStart[0]) {
                        robot.shooter.setTargetRPM(-1500);
                        robot.shooter.spin();
                        shooterReverseStart[0] = true;
                    }
                    return !shooterReverseStart[0]; // disables marker after stop fires
                })

                .afterTime(4.6, t -> {
                    if (!opModeIsActive()) return false;
                    if (!shooterReverseStop[0]) {
                        robot.shooter.setTargetRPM(shooterRPMClose);
                        robot.shooter.eStop();
                        shooterReverseStop[0] = true;
                    }
                    return !shooterReverseStop[0]; // disables marker after stop fires
                })


                //MOTIF 2
                .strafeToSplineHeading(new Vector2d(22, -15), Math.toRadians(270), new TranslationalVelConstraint(100))//go to motif
                .strafeToConstantHeading(new Vector2d(22, -69))//intake

                // ==============return============== \\
                .strafeToConstantHeading(new Vector2d(22, -32))//back up
                .strafeToSplineHeading(new Vector2d(-40,-24),Math.toRadians(230), new TranslationalVelConstraint(100))//shooting pose



                .build());

        robot.shooter.spin();
        robot.delay(2);
        robot.intake.intake();
        robot.delay(2);
        robot.shooter.eStop();
        robot.delay(2);
        robot.intake.stop();
        robot.shooter.eStop();




    }

    @Override
    public void onStop() {
        robot.stopAll();
    }
}