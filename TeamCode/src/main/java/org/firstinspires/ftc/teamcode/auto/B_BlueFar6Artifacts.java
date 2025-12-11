package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
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


    int shooterRPM = 2500;
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
                .strafeToLinearHeading(new Vector2d(56, -10), Math.toRadians(213))//go to shoot pose
                .build());

        AutoUtil.delay(2);

        Actions.runBlocking((t) -> {
            robot.transfer.spin();
            robot.intake.intake();
            return false;
        });
        AutoUtil.delay(.4);
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
                .afterTime(3, (t) -> {
                    robot.shooter.spin();
                    return false;
                })

                .strafeToSplineHeading(new Vector2d(24, -29), Math.toRadians(270))//go to motif 1
                .strafeToConstantHeading(new Vector2d(23, -66))//intake


                .strafeToLinearHeading(new Vector2d(56, -10), Math.toRadians(201.5))//go to shoot pose

                .build());


        //AutoUtil.delay(2);

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
                .afterTime(4.6, (t) -> {
                    robot.transfer.stop();
                    return false;
                })

                .afterTime(4.7, (t) -> {
                    robot.shooter.spin();
                    return false;
                })
                //get HP zone balls
                /*.strafeToSplineHeading(new Vector2d(42, 81), Math.toRadians(0))//line up for HP zone balls
                .strafeToSplineHeading(new Vector2d(61, 81), Math.toRadians(0))//line up for HP zone balls
                //.strafeToConstantHeading(new Vector2d(61, 65))//line up for HP zone balls
                .strafeToLinearHeading(new Vector2d(56, 10), Math.toRadians(162))//go to shoot pose
                 */


                .strafeToSplineHeading(new Vector2d(45, -70), Math.toRadians(290))//line up for HP zone balls
                .strafeToSplineHeading(new Vector2d(60, -75), Math.toRadians(315),  new TranslationalVelConstraint(25))//line up for HP zone balls
                .strafeToSplineHeading(new Vector2d(67, -75), Math.toRadians(315), new TranslationalVelConstraint(25))//line up for HP zone balls
                .strafeToSplineHeading(new Vector2d(60, -75), Math.toRadians(315))
                .strafeToLinearHeading(new Vector2d(56, -10), Math.toRadians(201.5))//go to shoot pose

                //.strafeToSplineHeading(new Vector2d(61, 81), Math.toRadians(0))//line up for HP zone balls



                .build());


        //AutoUtil.delay(2);

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




    }

    @Override
    public void onStop () {

    }


}