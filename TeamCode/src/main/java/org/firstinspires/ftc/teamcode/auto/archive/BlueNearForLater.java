package org.firstinspires.ftc.teamcode.auto.archive;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.util.AutoUtil;
import org.firstinspires.ftc.teamcode.subsystem.BaseRobot;

import org.firstinspires.ftc.teamcode.util.SampleAuto;

@Autonomous(name="BlueNearForLater")
public class BlueNearForLater extends SampleAuto {
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

        robot.LED.startOscillating();

    }

    @Override
    public void onStart() {


        while (opModeIsActive()) {
            Actions.runBlocking((t) -> {
                robot.shooter.spin();
                return false;
            });
            Actions.runBlocking(robot.drive.actionBuilder(robot.drive.getPose())
                    .strafeToSplineHeading(new Vector2d(-27, -24), Math.toRadians(227))//go to shooting pose
                    .build());


            Actions.runBlocking((t) -> {
                robot.intake.intake();
                return false;
            });
            Actions.runBlocking((t) -> {
                robot.transfer.spin();
                return false;
            });

            AutoUtil.delay(2.9);
            Actions.runBlocking((t) -> {
                robot.intake.stop();
                return false;
            });
            Actions.runBlocking((t) -> {
                robot.intake.setIntakePower(1);
                return false;
            });
            Actions.runBlocking((t) -> {
                robot.shooter.eStop();
                return false;
            });


            // ====================== Intake 1st Pile ====================== \\


            Actions.runBlocking((t) -> {
                robot.intake.setIntakePower(1);
                return false;
            });
            Actions.runBlocking(robot.drive.actionBuilder(robot.drive.getPose())

                    .afterTime(0, (t) -> {
                        robot.intake.intake();
                        //robot.transfer.spinReverse();
                        return false;
                    })

                    .afterTime(3, (t) -> {
                        robot.intake.stop();
                        //robot.transfer.stop();
                        return false;
                    })

                    .afterTime(3, (t) -> {
                        robot.transfer.spinReverse();
                        //robot.transfer.stop();
                        return false;
                    })
                    .afterTime(3.09, (t) -> {
                        robot.transfer.stop();
                        //robot.transfer.stop();
                        return false;
                    })

                    .afterTime(3.3, (t) -> {
                        robot.shooter.spin();
                        //robot.transfer.stop();
                        return false;
                    })

                    .strafeToSplineHeading(new Vector2d(-3, -20), Math.toRadians(270), new TranslationalVelConstraint(100))
                    .strafeToConstantHeading(new Vector2d(-3, -59), new TranslationalVelConstraint(30))


                    .strafeToConstantHeading(new Vector2d(-3, -57))
                    .strafeToSplineHeading(new Vector2d(-28, -20), Math.toRadians(221), new TranslationalVelConstraint(100))
                    .build());

            Actions.runBlocking((t) -> {robot.shooter.spin(); return false; });
            //AutoUtil.delay(2);
            Actions.runBlocking((t) -> {robot.intake.setIntakePower(1); return false;});
            Actions.runBlocking((t) -> {robot.intake.intake(); return false;});
            Actions.runBlocking((t) -> {robot.transfer.spin(); return false;});
            AutoUtil.delay(3);
            Actions.runBlocking((t) -> {robot.stopAll(); return false;});


            //---------------------PILE GRAB 2--------------------------------\\

            Actions.runBlocking(robot.drive.actionBuilder(robot.drive.getPose())
                    //start intaking
                    .afterTime(1.6, (t) -> {
                        robot.intake.intake();
                        robot.transfer.spinReverse();
                        return false;
                    })


                    //stop intaking
                    .afterTime(3.9, (t) -> {
                        robot.intake.stop();
                        robot.transfer.stop();
                        return false;
                    })

                    .afterTime(3.9, (t) -> {
                        robot.transfer.spinReverse();
                        //robot.transfer.stop();
                        return false;
                    })
                    .afterTime(3.99, (t) -> {
                        robot.transfer.stop();
                        //robot.transfer.stop();
                        return false;
                    })

                    .afterTime(4.2, (t) -> {
                        robot.shooter.spin();
                        //robot.transfer.stop();
                        return false;
                    })



                    //MOTIF 2
                    .strafeToSplineHeading(new Vector2d(22, -15), Math.toRadians(270), new TranslationalVelConstraint(100))//go to motif
                    .strafeToConstantHeading(new Vector2d(22, -60))//intake

                    // ==============return============== \\
                    .strafeToConstantHeading(new Vector2d(22, -32))//back up
                    .strafeToSplineHeading(new Vector2d(-45, -13), Math.toRadians(244), new TranslationalVelConstraint(100))//shooting pose
                    //                      this was changed thursday 11/13 from 240->244 ^

                    .build());

            Actions.runBlocking((t) -> {robot.shooter.spin(); return false; });
            //AutoUtil.delay(2);
            Actions.runBlocking((t) -> {robot.intake.intake(); return false; });
            Actions.runBlocking((t) -> {robot.transfer.spin(); return false; });
            AutoUtil.delay(2);
            Actions.runBlocking((t) -> {robot.shooter.eStop(); return false; });
            AutoUtil.delay(2);
            Actions.runBlocking((t) -> {robot.intake.stop(); return false; });
            Actions.runBlocking((t) -> {robot.shooter.eStop(); return false; });
            break;
        }
    }

    @Override
    public void onStop() {
        robot.stopAll();
    }
}