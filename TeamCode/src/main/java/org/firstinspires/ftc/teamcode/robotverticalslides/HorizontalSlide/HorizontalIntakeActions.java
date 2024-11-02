package org.firstinspires.ftc.teamcode.robotverticalslides.HorizontalSlide;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotverticalslides.constants.ConfigConstants;

public class HorizontalIntakeActions {
    public CRServo intakeServo;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private ElapsedTime runtime = new ElapsedTime();
    public HorizontalIntakeActions(Telemetry opModeTelemetry, HardwareMap opModeHardware) {
        this.telemetry = opModeTelemetry;
        this.hardwareMap = opModeHardware;

        intakeServo = hardwareMap.get(CRServo.class, ConfigConstants.HORIZONTAL_INTAKE);

        intakeServo.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    public void intake(double power) {
        intakeServo.setPower(-power);
    }
    public void outtake(double power) {
        intakeServo.setPower(power);
    }
    public void teleop(double intake, double outtake) {
        if (intake > 0.05) {
            intake(intake);
        } else if (outtake > 0.05) {
            outtake(outtake);
        } else {
            intake(0);
        }
    }
}
