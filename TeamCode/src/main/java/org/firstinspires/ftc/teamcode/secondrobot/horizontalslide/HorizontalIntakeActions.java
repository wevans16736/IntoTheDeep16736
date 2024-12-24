package org.firstinspires.ftc.teamcode.secondrobot.horizontalslide;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.secondrobot.constants.ConfigConstants;

public class HorizontalIntakeActions {
    public ServoImplEx intakeServo;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private ElapsedTime runtime = new ElapsedTime();
    public HorizontalIntakeActions(Telemetry opModeTelemetry, HardwareMap opModeHardware) {
        this.telemetry = opModeTelemetry;
        this.hardwareMap = opModeHardware;

        intakeServo = hardwareMap.get(ServoImplEx.class, ConfigConstants.HORIZONTAL_INTAKE);

        intakeServo.setPosition(0);
    }

    double openStartTime = 0;
    public void close() {
        closed = true;
    }
    public void open() {
        closed = false;
        openStartTime = System.currentTimeMillis();
    }
    //After opening, turn off the servo until it needs to close
    public void update() {
        if (!closed) {
            setPosition(0.2);
            if (System.currentTimeMillis() > openStartTime + 420) {
                if (intakeServo.isPwmEnabled()) {
//                    intakeServo.setPwmDisable();
                }
            }
        } else {
            setPosition(0.0);
            if (!intakeServo.isPwmEnabled()) {
                intakeServo.setPwmEnable();
            }
        }

    }
    public void setPosition(double position) {
        if (intakeServo.isPwmEnabled()) {
            intakeServo.setPosition(position);
        }
    }
    boolean wasInput = false;
    public boolean closed = false;
    public void teleop(boolean input) {
        if (input && !wasInput) {
            if (closed) {
                open();
            } else {
                close();
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
