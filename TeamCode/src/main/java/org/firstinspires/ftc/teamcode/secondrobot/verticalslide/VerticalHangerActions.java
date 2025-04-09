package org.firstinspires.ftc.teamcode.secondrobot.verticalslide;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Configuration.secondRobot.Pose;
import org.firstinspires.ftc.teamcode.secondrobot.constants.ConfigConstants;

public class VerticalHangerActions {
    public Servo verticalHanger = null;
    private HardwareMap hardwareMap;
    public VerticalHangerActions(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        verticalHanger = hardwareMap.get(Servo.class, ConfigConstants.VERTICAL_HANGER);

        verticalHanger.setPosition(Pose.verticalHangIn);
    }

    boolean hangerOut = false;
    public void setOut(boolean out) {
        hangerOut = out;
    }

    public void updateHanger() {
        if (hangerOut){
            verticalHanger.setPosition(Pose.verticalHangOut);
        } else {
            verticalHanger.setPosition(Pose.verticalHangIn);
        }
    }
    boolean wasActivate = false;
    public void teleOpHanger(boolean activate){
        if (activate &! wasActivate) {
            setOut(!hangerOut);
        }
        updateHanger();
        wasActivate = activate;
    }
}
