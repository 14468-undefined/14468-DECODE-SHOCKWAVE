package org.firstinspires.ftc.teamcode.subsystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auto.util.AutoUtil;
import org.firstinspires.ftc.teamcode.util.ColorfulTelemetry;

public class BaseRobot extends UndefinedSubsystemBase {


    public WebcamVisionSubsystem webcamVision;
    public AutoUtil autoGenerator;
    public ShooterSubsystem shooter;
    public DriveSubsystem drive;
    public IntakeSubsystem intake;
    public ColorfulTelemetry cTelemetry;
    public LEDSubsystem LED;

    public Telemetry telemetry;
    public HuskyLensSubsystem huskyLensVision;
    public TransferSubsystem transfer;


    public BaseRobot(HardwareMap hwMap, Pose2d startPos){

        drive = new DriveSubsystem(hwMap, startPos);
        shooter = new ShooterSubsystem(hwMap);
        intake = new IntakeSubsystem(hwMap, cTelemetry);
        autoGenerator = new AutoUtil(drive);
        webcamVision = new WebcamVisionSubsystem(hwMap);
        //huskyLensVision = new HuskyLensSubsystem(hwMap, cTelemetry);
        transfer = new TransferSubsystem(hwMap, cTelemetry);
        LED = new LEDSubsystem(hwMap, cTelemetry);


    }


    public void delay(double seconds){
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while(timer.seconds() < seconds){

        }

    }


    @Override
    public void printTelemetry(ColorfulTelemetry t) {
        //drive.printTelemetry(t);
        //intake.printTelemetry(t);
        //shooter.printTelemetry(telemetry);
        //webcamVision.printTelemetry(t);
        //huskyLensVision.printTelemetry(t);
        //transfer.printTelemetry(t);


    }

    @Override
    public void periodic() {
        //drive.periodic();
        intake.periodic();
        shooter.periodic();
        webcamVision.periodic();
        //huskyLensVision.periodic();
        transfer.periodic();
        LED.periodic();

    }

    public void stopAll(){
        intake.stop();
        shooter.eStop();
        //webcamVision.stopVision();
        transfer.stop();


    }

    class ExampleAction implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

           //do things here



            //AutoUtil.delay(1.5);


            return false;
        }
    }


}