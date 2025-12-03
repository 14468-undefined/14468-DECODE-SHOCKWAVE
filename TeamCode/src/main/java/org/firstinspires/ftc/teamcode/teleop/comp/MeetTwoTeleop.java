package org.firstinspires.ftc.teamcode.teleop.comp;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.LEDSubsystem;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.SampleCommandTeleop;

@TeleOp(name = "MeetThreeTeleop" , group = "AA - COMP")
public class MeetTwoTeleop extends SampleCommandTeleop {


    double driveSpeed = 1;

    boolean shooterOn = false;

    int shooterRPM = 2035;




    @Override
    public void onInit() {



        robot.drive.setDefaultCommand(new RunCommand(()-> robot.drive.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(g1.getLeftY() * driveSpeed, -g1.getLeftX() * driveSpeed), -g1.getRightX() * driveSpeed)), robot.drive));





        robot.shooter.setTargetRPM(shooterRPM);

        double shooterRealRPM = robot.shooter.getShooterVelocity();


    }

    @Override
    public void onStart() {






        g1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(() -> {
            driveSpeed = 1;
        });
        g1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(() -> {
            driveSpeed = .5;
        });
        g1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(() -> {
            driveSpeed = .2;
        });





        /*
        SHOOTING:
        g1 X - shoot 3;
        g1 B - cancel;

        DPAD UP = far zone
        DPAD Right = mid zone
        DPAD Down = close zone (ur never in close zone)
         */



        g2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenActive(() -> {
            robot.shooter.setTargetRPM(shooterRPM);
            robot.shooter.spin();
        });

        g2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenInactive(() -> {
            robot.shooter.eStop();
        });



        /*
        shooter reverse

        g2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenActive(() -> {
            robot.shooter.setTargetRPM(-1000);
            robot.shooter.spin();
        });
        g2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenInactive(() -> {
            robot.shooter.eStop();
        });


         */


        g2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenActive(() -> {
            robot.transfer.spin();
            robot.intake.intake();
        });
        g2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenInactive(() -> {
                    robot.intake.stop();
                    robot.transfer.stop();
        });


        g2.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).whenActive(() -> {
            robot.shooter.setTargetRPM(-6000);
            robot.shooter.spin();
        });
        g2.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).whenReleased(() -> {
            robot.shooter.setTargetRPM(shooterRPM);
            robot.shooter.eStop();
        });


        //right trigger = intake forwards, left trigger = intake reverse
        //g1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.05).whenActive(new InstantCommand(() -> robot.intake.intake())).whenInactive(new InstantCommand(() -> robot.intake.stop()));


        new Trigger(() -> gamepad1.right_trigger > .1).whenActive(new InstantCommand(() -> robot.intake.intake())).whenInactive(new InstantCommand(() -> robot.intake.stop()));
        new Trigger(() -> gamepad1.left_trigger > .1).whenActive(new InstantCommand(() -> robot.intake.intakeReverse())).whenInactive(new InstantCommand(() -> robot.intake.stop()));

        //new Trigger(() -> g1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.05).whenActive(new InstantCommand(() -> robot.intake.intakeReverse())).whenInactive(new InstantCommand(() -> robot.intake.stop()));

        g2.getGamepadButton(GamepadKeys.Button.X).whenPressed(() -> {
            //zone = 2;
            //numShots = 3;
            robot.intake.intake();
            //robot.transfer.spinReverse();

        });
        g2.getGamepadButton(GamepadKeys.Button.B).whenPressed(() -> {
            //zone = 3;
            //numShots = 3;

            robot.transfer.spinReverse();
        });

        g2.getGamepadButton(GamepadKeys.Button.X).whenReleased(() -> {
            //zone = 2;
            //numShots = 3;
            robot.intake.stop();
            robot.transfer.stop();

        });
        g2.getGamepadButton(GamepadKeys.Button.B).whenReleased(() -> {
            //zone = 3;
            //numShots = 3;

            robot.transfer.stop();
        });

        g2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenReleased(() -> {
            //zone = 3;
            //numShots = 3;
            robot.intake.intakeReverse();
            robot.transfer.stop();
        });
        g2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(() -> {
            //zone = 3;
            //numShots = 3;
            robot.intake.stop();
            robot.transfer.spinReverse();
        });







        g2.getGamepadButton(GamepadKeys.Button.A).whenPressed(() -> {
            //zone = 2;
            //numShots = 3;
            shooterRPM = 2035;
            robot.shooter.setTargetRPM(shooterRPM);

        });
        g2.getGamepadButton(GamepadKeys.Button.Y).whenPressed(() -> {
            //zone = 3;
            //numShots = 3;
            shooterRPM = 2485;
            robot.shooter.setTargetRPM(shooterRPM);
        });

        g2.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(() -> {
            //zone = 3;
            //numShots = 3;
            shooterRPM += 50;
            robot.shooter.setTargetRPM(shooterRPM);
        });
        g2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(() -> {
            //zone = 3;
            //numShots = 3;
            shooterRPM -= 50;
            robot.shooter.setTargetRPM(shooterRPM);
        });




    }

    @Override
    public void onLoop() {


        // Print intake telemetry every loopq
        pen.addData("Shooter RPM: ", robot.shooter.getShooterVelocity());
        pen.addData("Set RPM: ", robot.shooter.getTargetRPM());

        /*
         // === LED OSCILLATION SETUP ===
    double time = getRuntime();       // seconds since start
        double min = 0.28;
        double max = 0.72;

        double mid = (min + max) / 2.0;
        double range = (max - min) / 2.0;


    // Sine oscillation smoothly between min ↔ max
        double osc = mid + range * Math.sin(time * (2.0 * Math.PI / 5.0));
    // full wave every 5 sec


    // === LED LOGIC ===
    if (robot.shooter.isAtTargetSpeed()) {
        // Solid green when at speed
        robot.LED.setColor(LEDSubsystem.LEDColor.GREEN);

    } else if (robot.shooter.isActive() && robot.shooter.getShooterVelocity() > 100) {
        // Spooling up → use oscillation instead of fixed .28
        robot.LED.setColor(LEDSubsystem.LEDColor.RED);

    } else if (robot.shooter.getShooterVelocity() < 300) {
        // Idle → solid white
        robot.LED.setPoseTest(osc);
    }


         */

        if (robot.shooter.isAtTargetSpeed()) {
            // Solid green when at speed
            robot.LED.setColor(LEDSubsystem.LEDColor.GREEN);
            robot.LED.stopOscillating();

        } else if (robot.shooter.isActive() && robot.shooter.getShooterVelocity() > 100) {
            // Spooling up → use oscillation instead of fixed .28
            robot.LED.setColor(LEDSubsystem.LEDColor.RED);
            robot.LED.stopOscillating();

        } else if (robot.shooter.getShooterVelocity() < 300) {
            // Idle → solid white
            robot.LED.startOscillating();
        }
        else {
            robot.LED.stopOscillating();
            robot.LED.setPoseTest(1);
        }


        //pen.addLine("shooter RPM set:", shooterRPM);
    }

    @Override
    public void onStop() {
        robot.stopAll();
    }
}
