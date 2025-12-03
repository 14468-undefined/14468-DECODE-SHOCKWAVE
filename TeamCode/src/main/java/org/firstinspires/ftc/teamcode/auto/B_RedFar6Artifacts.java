package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.util.AutoUtil;
import org.firstinspires.ftc.teamcode.command.ShootCommand;
import org.firstinspires.ftc.teamcode.subsystem.BaseRobot;

import org.firstinspires.ftc.teamcode.util.SampleAuto;

@Autonomous(name="B_RedFar9Artifacts")
public class B_RedFar6Artifacts extends SampleAuto {
    private BaseRobot robot;
    private ShootCommand shoot3;


    int shooterRPM = 2500;
    @Override
    public void onInit() {
        robot = new BaseRobot(hardwareMap, new Pose2d(61, 18, Math.toRadians(180)));

        robot.shooter.setTargetRPM(shooterRPM);

        robot.LED.startOscillating();

    }

    @Override
    public void onStart() {



            Actions.runBlocking((t) -> {
                robot.shooter.spin();
                return false;
            });


            Actions.runBlocking(robot.drive.actionBuilder(robot.drive.getPose())
                    .strafeToLinearHeading(new Vector2d(56, 10), Math.toRadians(147))//go to shoot pose
                    .build());

            AutoUtil.delay(2);

            Actions.runBlocking((t) -> {
                robot.transfer.spin();
                robot.intake.intake();
                return false;
            });
            AutoUtil.delay(.7);
            Actions.runBlocking((t) -> {
                robot.intake.stop();
                robot.transfer.stop();
                return false;
            });
            AutoUtil.delay(.7);
            Actions.runBlocking((t) -> {
                robot.transfer.spin();
                robot.intake.intake();
                return false;
            });
            AutoUtil.delay(1);
            Actions.runBlocking((t) -> {
                robot.intake.stop();
                robot.transfer.stop();
                return false;
            });
            AutoUtil.delay(1);

            Actions.runBlocking((t) -> {
                robot.transfer.spin();
                robot.intake.intake();
                return false;
            });

            AutoUtil.delay(1);

            Actions.runBlocking((t) -> {
                robot.stopAll();
                return false;
            });
            // ====================== Intake 1st Pile ====================== \\


            //INTAKE PILE 1_________________________________________\\
            Actions.runBlocking(robot.drive.actionBuilder(robot.drive.getPose())


                    .afterTime(0, (t) -> {
                        robot.intake.intake();
                        return false;
                    })

                    .afterTime(4.8, (t) -> {
                        robot.intake.stop();
                        return false;
                    })

                    .afterTime(4.85, (t) -> {
                        robot.transfer.spinReverse();
                        return false;
                    })
                    .afterTime(4.91, (t) -> {
                        robot.transfer.stop();
                        return false;
                    })
                    .afterTime(5, (t) -> {
                        robot.shooter.spin();
                        return false;
                    })

                    .strafeToSplineHeading(new Vector2d(24, 29), Math.toRadians(90))//go to motif 1
                    .strafeToConstantHeading(new Vector2d(23, 66))//intake


                    .strafeToLinearHeading(new Vector2d(56, 10), Math.toRadians(158.5))//go to shoot pose

                    .build());


            AutoUtil.delay(2);

            Actions.runBlocking((t) -> {
                robot.transfer.spin();
                robot.intake.intake();
                return false;
            });
            AutoUtil.delay(.45);
            Actions.runBlocking((t) -> {
                robot.intake.stop();
                robot.transfer.stop();
                return false;
            });
            AutoUtil.delay(1);
            Actions.runBlocking((t) -> {
                robot.transfer.spin();
                robot.intake.intake();
                return false;
            });
            AutoUtil.delay(.6);
            Actions.runBlocking((t) -> {
                robot.intake.stop();
                robot.transfer.stop();
                return false;
            });
            AutoUtil.delay(1);

            Actions.runBlocking((t) -> {
                robot.transfer.spin();
                robot.intake.intake();
                return false;
            });

            AutoUtil.delay(.6);

            Actions.runBlocking((t) -> {
                robot.stopAll();
                return false;
            });
            // ====

            //INTAKE 2nd PILE IN CORNER
            Actions.runBlocking(robot.drive.actionBuilder(robot.drive.getPose())
                    .afterTime(0, (t) -> {
                        robot.intake.intake();
                        return false;
                    })

                    .afterTime(4.5, (t) -> {
                        robot.intake.stop();
                        return false;
                    })

                    .afterTime(4.55, (t) -> {
                        robot.transfer.spinReverse();
                        return false;
                    })
                    .afterTime(4.61, (t) -> {
                        robot.transfer.stop();
                        return false;
                    })

                    .afterTime(4.7, (t) -> {
                        robot.shooter.spin();
                        return false;
                    })
                    //get HP zone balls
                    .strafeToSplineHeading(new Vector2d(42, 81), Math.toRadians(0))//line up for HP zone balls
                    .strafeToSplineHeading(new Vector2d(61, 81), Math.toRadians(0))//line up for HP zone balls
                    //.strafeToConstantHeading(new Vector2d(61, 65))//line up for HP zone balls
                    .strafeToLinearHeading(new Vector2d(56, 10), Math.toRadians(162))//go to shoot pose

                    .build());


            Actions.runBlocking(robot.drive.actionBuilder(robot.drive.getPose())

                    .afterTime(0, (t) -> {
                        robot.intake.intake();
                        return false;
                    })

                    .afterTime(6, (t) -> {
                        robot.intake.stop();
                        return false;
                    })

                    .afterTime(6.1, (t) -> {
                        robot.transfer.spinReverse();
                        return false;
                    })
                    .afterTime(6.2, (t) -> {
                        robot.transfer.stop();
                        return false;
                    })
                    .afterTime(7, (t) -> {
                        robot.shooter.spin();
                        return false;
                    })
                    //.strafeToLinearHeading(new Vector2d(40, 67), Math.toRadians(0))

                    //.strafeToConstantHeading(new Vector2d(74, 67))

                    .build());


    }

        @Override
        public void onStop () {

        }


    }