package org.firstinspires.ftc.teamcode.Configuration.secondRobot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.GlobalVariables;
import org.firstinspires.ftc.teamcode.secondrobot.constants.ConfigConstants;

public class HorizontalSlideRR {
    Telemetry telemetry;
    private DcMotorEx horizontalSlideMotor = null;
    public HorizontalSlideRR(HardwareMap hardwareMap){
        horizontalSlideMotor = hardwareMap.get(DcMotorEx.class, ConfigConstants.HORIZONTAL_ARM);
        horizontalSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        horizontalSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalSlideMotor.setTargetPosition(0);
        horizontalSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        horizontalSlideMotor.setTargetPosition(0);
        horizontalSlideMotor.setVelocity(2000);
    }
    public class HorizontalSlideActions implements Action {
        private final int position;
        private boolean intizilize = false;
        public HorizontalSlideActions(int position){
            this.position = position;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(!intizilize) {
                horizontalSlideMotor.setTargetPosition(position);
                intizilize = true;
            }
            int pos = horizontalSlideMotor.getCurrentPosition();

//            if(pos > position){
//                Int = false;
//                return false;
//            }else {
//                return true;
//            }
//            return !(pos>position);
            return false;
        }
    }

    public Action horizontalSlideActions(int position) {return new HorizontalSlideActions(position);}

    public void setPose(int pose){
        horizontalSlideMotor.setTargetPosition(pose);
    }
    public double getSlideDistanceMath() {
        double encoderTicks = horizontalSlideMotor.getCurrentPosition();
        return Math.cos(Math.toRadians((encoderTicks / 7.0) - 70.0) * -1.0) * 13.0;
    }
    public int setSlideDistanceMath(double distance, double velocity) {
        double targetEncoderTics = 7.0 * (70.0 - Math.toDegrees(Math.acos(distance/13.0)));
       return (int) Range.clip(targetEncoderTics, 0, 650);
    }
    public int getSlideTick(){
        double distance = GlobalVariables.Y;
        int ticks = 0;

        return ticks;
    }

}
