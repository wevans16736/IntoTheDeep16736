package org.firstinspires.ftc.teamcode.robotverticalslides.HorizontalSlide;

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

    public void open() {
        open = true;
    }
    public void close() {
        open = false;
    }
    public void update() {
        if (open) {
            intakeServo.setPosition(0.0);
        } else {
            intakeServo.setPosition(0.2);
        }
    }
    boolean wasInput = false;
    public boolean open = false;
    public void teleop(boolean input) {
        if (input && !wasInput) {
            if (open) {
                close();
            } else {
                open();
            }
        }
        wasInput = input;
        update();
    }
    double position = 0.9;
    public void manual(boolean activate, boolean reverse) {
        if (activate) {
            if (!reverse) {
                position = position - 0.005;
            } else {
                position = position + 0.005;
            }
        }
        intakeServo.setPosition(position);
    }
}
