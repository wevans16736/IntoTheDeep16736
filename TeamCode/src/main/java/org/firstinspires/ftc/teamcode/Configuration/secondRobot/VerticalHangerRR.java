package org.firstinspires.ftc.teamcode.Configuration.secondRobot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.secondrobot.constants.ConfigConstants;

public class VerticalHangerRR {
    Servo verticalHanger;
    public VerticalHangerRR(HardwareMap hardwareMap){
        verticalHanger = hardwareMap.get(Servo.class, ConfigConstants.VERTICAL_HANGER);
        verticalHanger.setPosition(Pose.verticalHangIn);
    }

    public class VerticalHangAction implements Action {
        double position = 0.0;
        public VerticalHangAction(double position){this.position = position;}
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            verticalHanger.setPosition(position);
            return false;
        }
    }
    public Action verticalHangAction(double position){return new VerticalHangAction(position);}

    public void setPose(double pose){
        verticalHanger.setPosition(pose);
    }
}
