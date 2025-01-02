package org.firstinspires.ftc.teamcode.Configuration.secondRobot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.secondrobot.constants.ConfigConstants;

public class VerticalSlideRR {
    DcMotorEx verticalSlideMotorLeft;
    DcMotorEx verticalSlideMotorRight;
    public VerticalSlideRR(HardwareMap hardwareMap){
        verticalSlideMotorLeft = hardwareMap.get(DcMotorEx.class, ConfigConstants.VERTICAL_SLIDE1);
        verticalSlideMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        verticalSlideMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalSlideMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalSlideMotorLeft.setTargetPosition(0);
        verticalSlideMotorLeft.setVelocity(5000);

        verticalSlideMotorRight = hardwareMap.get(DcMotorEx.class, ConfigConstants.VERTICAL_SLIDE2);
        verticalSlideMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);
        verticalSlideMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalSlideMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalSlideMotorRight.setTargetPosition(0);
        verticalSlideMotorRight.setVelocity(5000);
    }

    public class VerticalSlideAction implements Action {
        int position;
        boolean intizilize = false;
        public VerticalSlideAction(int position){
            this.position = position;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(!intizilize){
                verticalSlideMotorLeft.setTargetPosition(position);
                verticalSlideMotorRight.setTargetPosition(position);
            }

            int pos = verticalSlideMotorLeft.getCurrentPosition();
            if(pos > position){
                intizilize = false;
                return false;
            }else {
                return true;
            }
        }
    }
    public Action verticalSlideAction(int position){return new VerticalSlideAction(position);}
}
