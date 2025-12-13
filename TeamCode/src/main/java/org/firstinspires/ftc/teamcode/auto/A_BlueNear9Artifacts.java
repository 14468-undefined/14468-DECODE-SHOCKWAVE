package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.util.AutoUtil;
import org.firstinspires.ftc.teamcode.subsystem.BaseRobot;

import org.firstinspires.ftc.teamcode.util.SampleAuto;

@Autonomous(name="A_BlueNear9Artifacts")
public class A_BlueNear9Artifacts extends SampleAuto {
    private BaseRobot robot;


    private int shooterRPMClose = 2180;//2135 //2100 //2090 //1990 //2200

    TelemetryPacket packet = new TelemetryPacket();

    Boolean gateDump = false;

    @Override
    public void onInit() {

        pen.addLine("waiting");
        if(gamepad1.x){
            gateDump = true;
            pen.addLine("GATE DUMP ADDED");
            telemetry.addLine("GATE DUMP ADDED");
        }
        pen.addLine("");
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



        while (opModeIsActive() && !isStopRequested()) {

            robot.LED.setPoseTest(.56);


            //robot.shooter.setTargetRPM(shooterRPMClose+85);
            Actions.runBlocking((t) -> {
                robot.shooter.spin();
                return false;
            });
            Actions.runBlocking(robot.drive.actionBuilder(robot.drive.getPose())
                    .strafeToSplineHeading(new Vector2d(-30, -27), Math.toRadians(232))//go to shooting pose
                    .build());


            Actions.runBlocking((t) -> {
                robot.intake.intake();
                return false;
            });
            Actions.runBlocking((t) -> {
                robot.transfer.spin();
                return false;
            });

            AutoUtil.delay(2.3);
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
            robot.shooter.setTargetRPM(shooterRPMClose);


            // ====================== Intake 1st Pile ====================== \\


            Actions.runBlocking((t) -> {
                robot.intake.setIntakePower(1);
                return false;
            });



            Actions.runBlocking(robot.drive.actionBuilder(robot.drive.getPose())

                    .afterTime(0, (t) -> {
                        robot.intake.intake();
                        robot.transfer.spinSlowReverse(.1);
                        //robot.transfer.spinReverse();
                        return false;
                    })

                    .afterTime(3.2, (t) -> {
                        //robot.intake.stop();
                        robot.intake.setIntakePower(.1);
                        robot.intake.intake();
                        //robot.transfer.stop();
                        return false;
                    })

                    .afterTime(3.4, (t) -> {
                        robot.transfer.spinReverse();
                        //robot.transfer.stop();
                        return false;
                    })
                    .afterTime(3.5, (t) -> {
                        robot.transfer.stop();
                        robot.intake.stop();
                        robot.intake.setIntakePower(1);
                        //robot.transfer.stop();
                        return false;
                    })

                    .afterTime(3.7, (t) -> {
                        robot.shooter.spin();
                        //robot.transfer.stop();
                        return false;
                    })

                    .strafeToSplineHeading(new Vector2d(-3, -20), Math.toRadians(270), new TranslationalVelConstraint(100))
                    .strafeToConstantHeading(new Vector2d(-3, -57), new TranslationalVelConstraint(25))


                    .strafeToConstantHeading(new Vector2d(-3, -55))

                    //NEW
                    //.strafeToConstantHeading(new Vector2d(7, -40), new TranslationalVelConstraint(30))
                    //.strafeToConstantHeading(new Vector2d(7, -55), new TranslationalVelConstraint(30))
                    //END NEW


                    .strafeToSplineHeading(new Vector2d(-26, -17), Math.toRadians(224), new TranslationalVelConstraint(100))
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
                        //robot.intake.stop();
                        robot.intake.setIntakePower(.1);
                        robot.intake.intake();
                        robot.transfer.stop();
                        return false;
                    })

                    .afterTime(3.9, ( t) -> {
                        robot.transfer.spinReverse();
                        //robot.transfer.stop();
                        return false;
                    })
                    .afterTime(3.99, (t) -> {
                        robot.transfer.stop();
                        robot.intake.stop();
                        robot.intake.setIntakePower(1);
                        //robot.transfer.stop();
                        return false;
                    })

                    .afterTime(4.2, (t) -> {
                        robot.shooter.spin();
                        //robot.transfer.stop();
                        return false;
                    })



                    //MOTIF 2
                    .strafeToSplineHeading(new Vector2d(22, -9), Math.toRadians(270), new TranslationalVelConstraint(100))//go to motif
                    .strafeToConstantHeading(new Vector2d(25, -65))//intake

                    // ==============return============== \\
                    .strafeToConstantHeading(new Vector2d(26, -32))//back up

                    .strafeToSplineHeading(new Vector2d(-36, -13), Math.toRadians(236))//go to shooting pose


                    .build());

            Actions.runBlocking((t) -> { robot.shooter.setTargetRPM(shooterRPMClose);robot.shooter.spin(); return false; });
            //AutoUtil.delay(2);
            Actions.runBlocking((t) -> {robot.intake.intake(); return false; });
            Actions.runBlocking((t) -> {robot.transfer.spin(); return false; });
            AutoUtil.delay(2);
            Actions.runBlocking((t) -> {robot.shooter.eStop(); return false; });
            Actions.runBlocking((t) -> {robot.intake.stop(); return false; });
            Actions.runBlocking((t) -> {robot.shooter.eStop(); return false; });

            /*Actions.runBlocking(robot.drive.actionBuilder(robot.drive.getPose())
                    .strafeToSplineHeading(new Vector2d(-55, -13), Math.toRadians(247), new TranslationalVelConstraint(100))//shooting pose
                    .build());

             */

            //super.stop();
            break;
        }
    }

    @Override
    public void onStop() {
        robot.stopAll();
    }
}