package org.firstinspires.ftc.teamcode.Configuration.secondRobot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.secondrobot.constants.ConfigConstants;

public class HorizontalRollRR {
    Servo HorizontalRollServo;

    public HorizontalRollRR(HardwareMap hardwareMap){
        HorizontalRollServo = hardwareMap.get(Servo.class, ConfigConstants.HORIZONTAL_ROLL);
        HorizontalRollServo.setPosition(ConfigurationSecondRobot.flat);
    }

    public class HorizontalRollAction implements Action{
        double position;
        public HorizontalRollAction(double position){
            this.position = position;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            HorizontalRollServo.setPosition(position);
            return false;
        }
    }
    public Action horizontalRollAction(double position){return new HorizontalRollAction(position);}

    public void setPose(double pose){
        HorizontalRollServo.setPosition(pose);
    }
}
