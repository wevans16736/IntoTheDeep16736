package org.firstinspires.ftc.teamcode.Autonomus.secondRobot.Championship;

import android.sax.StartElementListener;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalGrabberRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalRollRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalSlideRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalWristRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.Pose;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalGrabberRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalHangerRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalSlideRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalWristRR;
import org.firstinspires.ftc.teamcode.RR.GlobalVariables;
import org.firstinspires.ftc.teamcode.secondrobot.LimeSweet;

public class StrafeAction {
    DcMotorEx leftFront, leftRear, rightRear, rightFront;double grabX, grabY, angle = 0; Telemetry telemetry;
    VerticalSlideRR verticalSlideRR; VerticalWristRR verticalWristRR; VerticalGrabberRR verticalGrabberRR; VerticalHangerRR verticalHangerRR;
    HorizontalSlideRR horizontalSlideRR; HorizontalRollRR horizontalRollRR; HorizontalGrabberRR horizontalGrabberRR;
    HorizontalWristRR horizontalWristRR; LimeSweet lime;
    public StrafeAction(DcMotorEx leftFront, DcMotorEx leftBack, DcMotorEx rightBack, DcMotorEx rightFront, LimeSweet lime, Telemetry telemetry, HorizontalGrabberRR horizontalGrabberRR,
                        HorizontalRollRR horizontalRollRR, HorizontalSlideRR horizontalSlideRR,
                        HorizontalWristRR horizontalWristRR, VerticalGrabberRR verticalGrabberRR,
                        VerticalHangerRR verticalHangerRR, VerticalSlideRR verticalSlideRR,
                        VerticalWristRR verticalWristRR){
        this.leftFront = leftFront;
        this.leftRear = leftBack;
        this.rightRear = rightBack;
        this.rightFront = rightFront;
        this.lime = lime;
        this.verticalSlideRR = verticalSlideRR;
        this.verticalWristRR = verticalWristRR;
        this.verticalGrabberRR = verticalGrabberRR;
        this.verticalHangerRR = verticalHangerRR;
        this.horizontalSlideRR = horizontalSlideRR;
        this.horizontalRollRR = horizontalRollRR;
        this.horizontalGrabberRR = horizontalGrabberRR;
        this.horizontalWristRR = horizontalWristRR;
        this.telemetry = telemetry;
    }
    public void grabButter(){
        horizontalGrabberRR.setPose(Pose.horizontalGrabberWide);
        horizontalRollRR.setPose((angle / 90) * 0.3);
        horizontalWristRR.setPose(Pose.horizontalWristIntake);
//        setSlideDistanceMath(grabY, 3000);
        strafeDistance(grabX + 1);
    }
    public double roll(){
        return ((angle / 90) * .3);
    }

    public void strafeDistance(double distance) {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double ticksPerCM = 12;

        int totalTicks = (int) (ticksPerCM * distance);
        // When going left, the target position needs to be inverted from when we go right
        leftFront.setTargetPosition(-totalTicks);
        rightFront.setTargetPosition(totalTicks);
        leftRear.setTargetPosition(totalTicks);
        rightRear.setTargetPosition(-totalTicks);

        // Switch to RUN_TO_POSITION mode
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double speed = 2000;
        leftFront.setVelocity(-speed);
        rightFront.setVelocity(-speed);
        leftRear.setVelocity(-speed);
        rightRear.setVelocity(-speed);
    }
    public void setMode(){
        // Switch to Run_WIthout_Position mode
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void getButterPose() {
        grabX = -lime.scanButter().get(0);
        grabY = lime.scanButter().get(1);
        angle = -lime.scanButter().get(2) + 180;
        telemetry.addData("Original-grabX", grabX);
        telemetry.addData("Original-grabY", grabY);
        telemetry.addData("Original-grabAngle", angle);
        if (angle > 170) {
            angle -= 180;
        }
        grabX -= 3 * Math.cos(Math.toRadians(90 + angle));
        grabY += -3 * Math.sin(Math.toRadians(90 + angle)) + 2;
        telemetry.addData("grabX", grabX);
        telemetry.addData("grabY", grabY);
        telemetry.addData("grabAngle", angle);
        telemetry.update();
    }
    public int setSlideDistanceMath() {
        //Based off law of sines
        double linkageLength = 27;
        double targetRads = Math.acos(grabX / (2 * linkageLength));
        //based off of endpoints of rad 1.31812/ticks 0 and rad 0/ticks 640
        double targetTicks = -(640/0.842234) * (targetRads - 1.31812);
        return ((int) Range.clip(targetTicks, 0, 650));
//        horizontalSlideRR.setPose(((int) Range.clip(targetTicks, 0, 650)));
//        horizontalSlideRR.setVelocity(velocity);
    }
}
