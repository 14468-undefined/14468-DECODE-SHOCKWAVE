package org.firstinspires.ftc.teamcode.teleop.helper;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystem.LEDSubsystem;
import org.firstinspires.ftc.teamcode.util.ColorfulTelemetry;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.SampleCommandTeleop;

@TeleOp(name = "PresentationTeleop" , group = "helper")
public class PresentationTeleop extends SampleCommandTeleop {




    boolean shooterOn = false;

    int shooterRPM = 2135;



    ElapsedTime time = new ElapsedTime();

    ElapsedTime scriptTimer = new ElapsedTime();
    double startPos = 0.0;
    double endPos   = 1.0;
    double duration = 10;     // seconds for each sweep

    boolean goingUp = true;    // whether we're going 0→1 or 1→0

    @Override
    public void onInit() {







        robot.shooter.setTargetRPM(shooterRPM);//setlower

        double shooterRealRPM = robot.shooter.getShooterVelocity();


        robot.intake.setIntakePower(1);//low

       // robot.LED.startOscillating();
    }

    @Override
    public void onStart() {









        g2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenActive(() -> {
            robot.shooter.setTargetRPM(shooterRPM);
            robot.shooter.spin();
        });

        g2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenInactive(() -> {
            robot.shooter.eStop();
        });



        TriggerReader leftTriggerReader = new TriggerReader(
                g2, GamepadKeys.Trigger.LEFT_TRIGGER
        );
        if (leftTriggerReader.isDown()){
            robot.transfer.spinReverse();
        }
        if(leftTriggerReader.wasJustReleased()){
            robot.transfer.stop();
        }

        TriggerReader rightTriggerReader = new TriggerReader(
                g2, GamepadKeys.Trigger.RIGHT_TRIGGER
        );
        if (rightTriggerReader.isDown()){
            robot.intake.intake();
        }
        if(rightTriggerReader.wasJustReleased()){
            robot.intake.stop();
        }



        new Trigger(() -> gamepad2.right_trigger > .1).whenActive(new InstantCommand(() -> robot.intake.intake())).whenInactive(new InstantCommand(() -> robot.intake.stop()));
        new Trigger(() -> gamepad2.left_trigger > .1).whenActive(new InstantCommand(() -> robot.intake.intakeReverse())).whenInactive(new InstantCommand(() -> robot.intake.stop()));


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


        g2.getGamepadButton(GamepadKeys.Button.X).whenPressed(() -> {
            //zone = 2;
            //numShots = 3;
            robot.intake.intake();
            //robot.transfer.spinReverse();

        });
        g2.getGamepadButton(GamepadKeys.Button.B).whenPressed(() -> {
            //zone = 3;
            //numShots = 3;
            robot.intake.intakeReverse();
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
            robot.intake.stop();
            robot.transfer.stop();
        });











    }


    @Override
    public void onLoop() {
//        double t = time.seconds();
//        double sweepTime = duration; // time for 0->1
//        double cycleTime = sweepTime * 2;
//        double cyclePos = (t % cycleTime) / sweepTime; // 0→2
//
//        double servoPos;
//        if (cyclePos <= 1.0) {
//            servoPos = cyclePos;  // 0 -> 1
//        } else {
//            servoPos = 2 - cyclePos; // 1 -> 0
//        }
//
//        robot.LED.setPoseTest(servoPos);




        double timePassed = scriptTimer.seconds();
        pen.addData("Time: ", timePassed);
        if(timePassed < 240) {//4 min
            pen.setColor(ColorfulTelemetry.Green);
        }
        else{
            pen.setColor(ColorfulTelemetry.Red);
        }
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
    }




    @Override
    public void onStop() {
        robot.stopAll();
        robot.LED.stopOscillating();
    }
}
