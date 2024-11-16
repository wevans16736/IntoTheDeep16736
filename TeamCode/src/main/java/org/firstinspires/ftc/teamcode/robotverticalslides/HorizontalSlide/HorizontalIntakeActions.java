package org.firstinspires.ftc.teamcode.robotverticalslides.HorizontalSlide;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotverticalslides.constants.ConfigConstants;

public class HorizontalIntakeActions {
    public Servo intakeServo;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private ElapsedTime runtime = new ElapsedTime();
    public HorizontalIntakeActions(Telemetry opModeTelemetry, HardwareMap opModeHardware) {
        this.telemetry = opModeTelemetry;
        this.hardwareMap = opModeHardware;

        intakeServo = hardwareMap.get(Servo.class, ConfigConstants.HORIZONTAL_INTAKE);

        intakeServo.setPosition(0);
    }
    public void intake(double power) {
        intakeServo.setPosition(0.2 - power * 0.2);
    }
    public void outtake(double power) {
//        intakeServo.setPosition(power);
//        telemetry.addData("intake position", intakeServo.getPosition());
    }
    public void teleop(double intake, double outtake) {
        if (intake > 0.05) {
            intake(intake);
        } else if (outtake > 0.05) {
            outtake(outtake);
        } else {
            intake(0.2);
        }
    }
}
