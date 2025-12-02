package org.firstinspires.ftc.teamcode.subsystem;




import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.ColorfulTelemetry;
import org.firstinspires.ftc.teamcode.util.Constants;

public class TransferSubsystem extends SubsystemBase {




    private final MotorEx transfer;
    double transferPower = .6;//was .6
    double reverseTransferPower = 1;

    private ColorfulTelemetry cTelemetry;
    private HardwareMap hardwareMap;
    public TransferSubsystem(HardwareMap hardwareMap, ColorfulTelemetry telemetry) {

        // ================== MOTORS ================== \\
        this.cTelemetry = telemetry;

        transfer = new MotorEx(hardwareMap, "transfer");

        transfer.setRunMode(MotorEx.RunMode.RawPower);
        transfer.setInverted(true);//run forwards

        transfer.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.FLOAT);

    }



    public void setTransferPower(double p) {
        transferPower = p;
    }

    public void setReverseIntakePower(double p) {
        reverseTransferPower = p;
    }

    public double getTransferPower() {
        return transferPower;
    }

    public double getReverseTransferPower() {
        return reverseTransferPower;
    }

    public void spin() {
        transfer.set(transferPower);
    }

    //send balls out through intake
    public void spinReverse() {
        transfer.set(-reverseTransferPower);
    }

    public void spinSlowReverse(double p){
        transfer.set(-p);
    }

    public void stop() {
        transfer.set(0);
    }
    public void printTelemetry(ColorfulTelemetry t) {

        t.reset(); // reset any previous styles


        t.setColor(ColorfulTelemetry.Black).bold();
        t.addLine("INTAKE SUBSYSTEM");






        t.update();

    }

    @Override
    public void periodic() {



    }


}
