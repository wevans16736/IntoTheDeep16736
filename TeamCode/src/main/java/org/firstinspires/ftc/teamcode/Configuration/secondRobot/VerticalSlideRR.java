package org.firstinspires.ftc.teamcode.Configuration.secondRobot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.secondrobot.constants.ConfigConstants;

public class VerticalSlideRR {
    DcMotorEx verticalSlideMotorLeft;
    DcMotorEx verticalSlideMotorRight;
    TouchSensor magnetSwitch;
    public VerticalSlideRR(HardwareMap hardwareMap){
//        magnetSwitch = hardwareMap.get(TouchSensor.class, ConfigConstants.VERTICAL_SWITCH);
        verticalSlideMotorLeft = hardwareMap.get(DcMotorEx.class, ConfigConstants.VERTICAL_SLIDE1);
        verticalSlideMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        verticalSlideMotorLeft.setPower(1);
        verticalSlideMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalSlideMotorLeft.setTargetPosition(0);
        verticalSlideMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalSlideMotorLeft.setTargetPosition(0);
        verticalSlideMotorLeft.setVelocity(5000);

        verticalSlideMotorRight = hardwareMap.get(DcMotorEx.class, ConfigConstants.VERTICAL_SLIDE2);
        verticalSlideMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);
        verticalSlideMotorRight.setPower(1);
        verticalSlideMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalSlideMotorRight.setTargetPosition(0);
        verticalSlideMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        verticalSlideMotorRight.setTargetPosition(0);
        verticalSlideMotorRight.setVelocity(5000);

//        while(!magnetSwitch.isPressed()){
//            verticalSlideMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            verticalSlideMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            verticalSlideMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            verticalSlideMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            verticalSlideMotorLeft.setPower(-0.5);
//            verticalSlideMotorRight.setPower(-0.5);
//        }

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
//            if(pos > position){
//                intizilize = false;
//                return false;
//            }else {
//                return true;
            return false;
//            }
        }
    }
    public Action verticalSlideAction(int position){return new VerticalSlideAction(position);}

    public void setPose(int pose){
        verticalSlideMotorRight.setTargetPosition(pose);
        verticalSlideMotorLeft.setTargetPosition(pose);
    }
    public int returnPose(){
        return verticalSlideMotorLeft.getCurrentPosition();
    }
}
