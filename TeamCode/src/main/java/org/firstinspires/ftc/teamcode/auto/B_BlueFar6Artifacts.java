package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.util.AutoUtil;
import org.firstinspires.ftc.teamcode.command.ShootCommand;
import org.firstinspires.ftc.teamcode.subsystem.BaseRobot;

import org.firstinspires.ftc.teamcode.util.SampleAuto;

@Autonomous(name="B_BlueFar9Artifacts")
public class B_BlueFar6Artifacts extends SampleAuto {
    private BaseRobot robot;
    private ShootCommand shoot3;



    int shooterRPM = 2800;
    @Override
    public void onInit() {
        robot = new BaseRobot(hardwareMap, new Pose2d(61, -18, Math.toRadians(180)));
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
                    .strafeToLinearHeading(new Vector2d(56, -10), Math.toRadians(204))//go to shoot pose
                    .build());

            AutoUtil.delay(2);

            Actions.runBlocking((t) -> {
                robot.transfer.spin();
                robot.intake.intake();
                return false;
            });
            AutoUtil.delay(.5);
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
            AutoUtil.delay(.5);
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

            AutoUtil.delay(.5);

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

                    .strafeToSplineHeading(new Vector2d(40, -29), Math.toRadians(270))//go to motif 1
                    .strafeToConstantHeading(new Vector2d(40, -66))//intake


                    .strafeToLinearHeading(new Vector2d(59, -10), Math.toRadians(199))//go to shoot pose

                    .build());


            AutoUtil.delay(.5);

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
                .strafeToLinearHeading(new Vector2d(45, -64), Math.toRadians(0))

                .strafeToConstantHeading(new Vector2d(74, -64))

                .build());

        AutoUtil.delay(2);

            Actions.runBlocking(robot.drive.actionBuilder(robot.drive.getPose())
                            .strafeToConstantHeading(new Vector2d(59, -28))
                            .build());




    }

    @Override
    public void onStop () {

    }


}