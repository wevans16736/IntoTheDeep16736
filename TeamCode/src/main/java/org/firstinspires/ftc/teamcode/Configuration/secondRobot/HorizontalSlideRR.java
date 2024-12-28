package org.firstinspires.ftc.teamcode.Configuration.secondRobot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.secondrobot.constants.ConfigConstants;

public class HorizontalSlideRR {
    private DcMotorEx horizontalSlideMotor = null;
    public HorizontalSlideRR(HardwareMap hardwareMap){
        horizontalSlideMotor = hardwareMap.get(DcMotorEx.class, ConfigConstants.HORIZONTAL_ARM);
        horizontalSlideMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        horizontalSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalSlideMotor.setTargetPosition(0);
        horizontalSlideMotor.setVelocity(3000);
        horizontalSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public class HorizontalSlideActions implements Action {
        private int position;
        private boolean Int = false;
        public HorizontalSlideActions(int position){
            this.position = position;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(!Int) {
                horizontalSlideMotor.setTargetPosition(position);
                Int = true;
            }
            int pos = horizontalSlideMotor.getCurrentPosition();

            return !(pos>position);
        }
    }
    public Action horizontalSlideActions(int position) {return new HorizontalSlideActions(position);}
}
