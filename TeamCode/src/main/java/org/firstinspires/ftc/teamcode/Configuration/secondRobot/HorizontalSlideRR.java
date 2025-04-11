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
import org.firstinspires.ftc.teamcode.RR.GlobalVariables;
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
    private class HorizontalSlideActions implements Action {
        private final int position;
        private boolean intizilize = false;
        public HorizontalSlideActions(int position){
            this.position = position;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            horizontalSlideMotor.setTargetPosition(position);
            return false;
        }
    }

    public Action horizontalSlideActions(int position) {return new HorizontalSlideActions(position);}
//    public Action horizontalSlideDistance(double distance, double velocity){return new HorizontalSlideDistance(distance, velocity);}

    public void setPose(int pose){
        horizontalSlideMotor.setTargetPosition(pose);
    }
    public double getSlideDistanceMath() {
        double encoderTicks = horizontalSlideMotor.getCurrentPosition();
        return Math.cos(Math.toRadians((encoderTicks / 7.0) - 70.0) * -1.0) * 13.0;
    }

    public void setSlideDistanceMath(double distance, double velocity) {
        //Based off law of sines
        double linkageLength = 27;
        double targetRads = Math.acos(distance / (2 * linkageLength));
        //based off of endpoints of rad 1.31812/ticks 0 and rad 0/ticks 640
        double targetTicks = -(640/0.842234) * (targetRads - 1.31812);
        horizontalSlideMotor.setTargetPosition((int) Range.clip(targetTicks, 0, 650));
        horizontalSlideMotor.setVelocity(velocity);
    }
//    public int setSlideDistanceMath(double distance, double velocity) {
//        double targetEncoderTics = 7.0 * (70.0 - Math.toDegrees(Math.acos(distance/13.0)));
//       return (int) Range.clip(targetEncoderTics, 0, 650);
//    }
    public int getSlideTick(){
        double distance = GlobalVariables.Y;
        int ticks = 0;

        return ticks;
    }

}
