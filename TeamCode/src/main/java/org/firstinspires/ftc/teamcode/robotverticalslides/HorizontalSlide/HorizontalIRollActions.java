package org.firstinspires.ftc.teamcode.robotverticalslides.HorizontalSlide;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Autonomus.Configuration;
import org.firstinspires.ftc.teamcode.robotverticalslides.constants.ConfigConstants;

public class HorizontalIRollActions {
    public ServoImplEx rollServo;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private ElapsedTime runtime = new ElapsedTime();
    private double flat = Configuration.flat;

    public HorizontalIRollActions(Telemetry opModeTelemetry, HardwareMap opModeHardware) {
        this.telemetry = opModeTelemetry;
        this.hardwareMap = opModeHardware;
        rollServo = hardwareMap.get(ServoImplEx.class, ConfigConstants.HORIZONTAL_ROLL);
        rollServo.setPosition(flat);
    }

    double prevPos = 0;
    double changePosStartTime = 0;
    public void setPosition(double position) {
        //Sets the position. If the position hasn't been changed in a while, turn off the servo. Turn it back on when the position changes
        if (prevPos != position) {
            if (!rollServo.isPwmEnabled()) {
                rollServo.setPwmEnable();
            }
            changePosStartTime = System.currentTimeMillis();
            position = Math.abs(position);
            rollServo.setPosition(position + Configuration.flat);
        }
        if (System.currentTimeMillis() > changePosStartTime + 420 && rollServo.isPwmEnabled()) {
            rollServo.setPwmDisable();
            //this part is just so it doesn't check if pwm is enabled for another little while
            changePosStartTime = System.currentTimeMillis();
        }
        prevPos = position;
    }
    boolean forward = true;
    boolean wasOperate = false;
    public void teleOp(boolean operate){
        if (operate && !wasOperate) {
            forward = !forward;
        }
        if (forward) {
            setPosition(0.3);
        } else {
            setPosition(0);
        }
        wasOperate = operate;
    }
}