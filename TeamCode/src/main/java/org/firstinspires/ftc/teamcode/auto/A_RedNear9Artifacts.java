package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.util.AutoUtil;
import org.firstinspires.ftc.teamcode.command.ShootCommand;
import org.firstinspires.ftc.teamcode.subsystem.BaseRobot;

import org.firstinspires.ftc.teamcode.util.SampleAuto;

@Autonomous(name="RedNear9Artifacts")
public class A_RedNear9Artifacts extends SampleAuto {
    private BaseRobot robot;
    private ShootCommand shoot3;

    private int shooterRPMClose = 2135;//2135 //2100

    TelemetryPacket packet = new TelemetryPacket();

    @Override
    public void onInit(){
        
        robot = new BaseRobot(hardwareMap, new Pose2d(-61, 40, Math.toRadians(180)));

        robot.shooter.setTargetRPM(shooterRPMClose);

        robot.intake.setIntakePower(.8);

        packet.put("target_shooter_rpm", robot.shooter.getTargetRPM());
        packet.put("current_shooter_rpm", robot.shooter.getShooterVelocity());


        while(!opModeIsActive()){
            robot.LED.startOscillating();
        }

    }

    @Override
    public void onStart() {


        while (opModeIsActive()) {



                /*
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
                        try {
                            Thread.sleep(40);
                        } catch (InterruptedException e) {
                        }
                    }
                }).start();

             */


                Actions.runBlocking((t) -> {
                    robot.shooter.spin();
                    return false;
                });
                Actions.runBlocking(robot.drive.actionBuilder(robot.drive.getPose())
                        .strafeToSplineHeading(new Vector2d(-27, 24), Math.toRadians(131))//go to shooting pose
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

                robot.intake.setIntakePower(1);
                Actions.runBlocking(robot.drive.actionBuilder(robot.drive.getPose())

                        .afterTime(0, t -> {
                            robot.intake.intake();
                            return false;
                        })

                        .afterTime(2.1, t -> {//was 3.7

                            robot.intake.stop();
                            return false;
                        })

                        .afterTime(3.1, (t) -> {
                            robot.transfer.spinReverse();
                            //robot.transfer.stop();
                            return false;
                        })
                        .afterTime(3.2, (t) -> {
                            robot.transfer.stop();
                            //robot.transfer.stop();
                            return false;
                        })

                        .afterTime(3.4, (t) -> {
                            robot.shooter.spin();
                            return false;
                        })


                        .strafeToSplineHeading(new Vector2d(-17, 25.4), Math.toRadians(90))
                        .strafeToConstantHeading(new Vector2d(-19, 64.3), new TranslationalVelConstraint(30))


                        .strafeToConstantHeading(new Vector2d(-24, 57))
                        .strafeToSplineHeading(new Vector2d(-30, 24), Math.toRadians(145))
                        .build());

                Actions.runBlocking((t) -> {
                    robot.shooter.spin();
                    return false;
                });
                //AutoUtil.delay(2);
                Actions.runBlocking((t) -> {
                    robot.intake.setIntakePower(1);
                    return false;
                });
                Actions.runBlocking((t) -> {
                    robot.intake.intake();
                    return false;
                });
                Actions.runBlocking((t) -> {
                    robot.transfer.spin();
                    return false;
                });
                AutoUtil.delay(3);
                Actions.runBlocking((t) -> {
                    robot.stopAll();
                    return false;
                });
                //wait til shoot 3 is done to move

                Actions.runBlocking((t) -> {
                   robot.shooter.setTargetRPM(shooterRPMClose-40);
                   return false;
                });

                Actions.runBlocking(robot.drive.actionBuilder(robot.drive.getPose())

                        .afterTime(1.6, t -> {

                            robot.intake.intake();
                            return false; // becomes false after first trigger
                        })

                        .afterTime(3.9, t -> {

                            robot.intake.stop();
                            return false;
                        })


                        .afterTime(4, (t) -> {
                            robot.transfer.spinReverse();
                            //robot.transfer.stop();
                            return false;
                        })
                        .afterTime(4.09, (t) -> {
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
                        .strafeToSplineHeading(new Vector2d(6, 22), Math.toRadians(88))//go to motif
                        .strafeToConstantHeading(new Vector2d(7, 69))//intake

                        // ==============return============== \\
                        .strafeToConstantHeading(new Vector2d(9, 50))//back up


                        //.strafeToSplineHeading(new Vector2d(-43, 24), Math.toRadians(139))//shooting pose
                        .strafeToLinearHeading(new Vector2d(-43, 24), Math.toRadians(132))//shooting pose



                        .build());


                Actions.runBlocking((t) -> {
                    robot.shooter.spin();
                    return false;
                });
                //AutoUtil.delay(2);
                Actions.runBlocking((t) -> {
                    robot.intake.intake();
                    return false;
                });
                Actions.runBlocking((t) -> {
                    robot.transfer.spin();
                    return false;
                });
                AutoUtil.delay(2);
                Actions.runBlocking((t) -> {
                    robot.shooter.eStop();
                    return false;
                });
                AutoUtil.delay(2);
                Actions.runBlocking((t) -> {
                    robot.intake.stop();
                    return false;
                });
                Actions.runBlocking((t) -> {
                    robot.shooter.eStop();
                    return false;
                });
                break;

        }
    }

    @Override
    public void onStop() {
        robot.stopAll();
    }
}