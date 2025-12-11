package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.util.ColorfulTelemetry;
import org.firstinspires.ftc.teamcode.util.Constants;

public class IntakeSubsystem extends UndefinedSubsystemBase {

    private final MotorEx intakeMotor;

    private ColorfulTelemetry cTelemetry;
    private HardwareMap hardwareMap;

    //power - adjusted in teleop but this is default
    private double intakePower = Constants.intakeConstants.INTAKE_POWER;//1
    private double reverseIntakePower = Constants.intakeConstants.REVERSE_INTAKE_POWER;//.5

    public IntakeSubsystem(HardwareMap hardwareMap, ColorfulTelemetry telemetry) {
        this.cTelemetry = telemetry;

        intakeMotor = new MotorEx(hardwareMap, "intake");


        //power instead of vel control
        intakeMotor.setRunMode(MotorEx.RunMode.RawPower);
        intakeMotor.setInverted(false);//run forwards
    }

    //setters for tuning power
    public void setIntakePower(double p) {
        intakePower = p;
    }

    public void setReverseIntakePower(double p) {
        reverseIntakePower = p;
    }

    public double getIntakePower() {
        return intakePower;
    }

    public double getReverseIntakePower() {
        return reverseIntakePower;
    }

    public void intake() {
        intakeMotor.set(intakePower);
    }

    //send balls out through intake
    public void intakeReverse() {
        intakeMotor.set(-reverseIntakePower);
    }

    public void stop() {
        intakeMotor.set(0);
    }

    public void printTelemetry(ColorfulTelemetry t) {
        t.reset(); // reset any previous styles

        t.setColor(ColorfulTelemetry.Blue).bold().addLine("INTAKE SUBSYSTEM").reset();

        String status;
        if (intakeMotor.get() > 0) {
            status = "Intaking";
            t.setColor(ColorfulTelemetry.Green);
        } else if (intakeMotor.get() < 0) {
            status = "Reversing";
            t.setColor(ColorfulTelemetry.Orange);
        } else {
            status = "Stopped";
            t.setColor(ColorfulTelemetry.Red);
        }
        t.addData("Status", status);

        t.update();

        //return power
        t.addData("Motor Power", intakeMotor.get());
        t.addData("Intake Power Setting", intakePower);
        t.addData("Outtake Power Setting", reverseIntakePower);
        t.update();
    }

    @Override
    public void periodic() {

    }
}
