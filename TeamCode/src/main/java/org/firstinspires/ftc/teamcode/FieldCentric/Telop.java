package org.firstinspires.ftc.teamcode.FieldCentric;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalGrabberRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalRollRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalSlideRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalWristRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalHangerRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalSlideRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalWristRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalGrabberRR;
import org.firstinspires.ftc.teamcode.GlobalVariables;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.secondrobot.constants.ConfigConstants;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "FieldCentric")
public class Telop extends OpMode {
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();
    VerticalSlideRR verticalSlide;
    VerticalWristRR verticalWrist;
    VerticalGrabberRR verticalGrabber;
    VerticalHangerRR verticalHanger;
    HorizontalSlideRR horizontalSlide;
    HorizontalRollRR horizontalRoll;
    HorizontalGrabberRR horizontalGrabber;
    HorizontalWristRR horizontalWrist;
    DriveTrain driveTrain;
    Pose2d currentPose;
    Attachment attachment;
    @Override
    public void init() {
        //set up Pinpoint and Pose2d class
        Pose2d pose;
        PinpointDrive drive;

        //all of these class is under Configuration.secondRobot
        verticalSlide = new VerticalSlideRR(hardwareMap);
        verticalWrist = new VerticalWristRR(hardwareMap , true);
        verticalGrabber = new VerticalGrabberRR(hardwareMap);

        horizontalSlide = new HorizontalSlideRR(hardwareMap);
        horizontalRoll = new HorizontalRollRR(hardwareMap);
        horizontalGrabber = new HorizontalGrabberRR(hardwareMap);
        horizontalWrist = new HorizontalWristRR(hardwareMap);
        verticalHanger = new VerticalHangerRR(hardwareMap);
        attachment = new Attachment(verticalSlide, verticalWrist, verticalGrabber,
                horizontalSlide, horizontalRoll, horizontalGrabber, horizontalWrist, verticalHanger,
                runningActions, dash, System.currentTimeMillis());

        //setup the drive train
        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, ConfigConstants.FRONT_LEFT);
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, ConfigConstants.FRONT_RIGHT);
        DcMotorEx rearLeft = hardwareMap.get(DcMotorEx.class, ConfigConstants.BACK_LEFT);
        DcMotorEx rearRight = hardwareMap.get(DcMotorEx.class, ConfigConstants.BACK_RIGHT);
        IMU imu = hardwareMap.get(IMU.class, ConfigConstants.IMU);
        driveTrain = new DriveTrain(frontLeft, frontRight, rearLeft, rearRight, imu);
        if(!GlobalVariables.autoStarted){
            this.currentPose = new Pose2d(0,0,0);
        } else{
            this.currentPose = GlobalVariables.currentPose;
        }
        GlobalVariables.autoStarted = false;
    }
    boolean percise = false;
    @Override
    public void loop() {
        //update time
        attachment.updateTime(System.currentTimeMillis());
        //verticalGrabber
        attachment.verticalGrabber(gamepad1.right_bumper);
        //horizontal slide
        attachment.horizontalSlide(gamepad1.triangle);
        //horizontal grabber
        attachment.horizontalGrabber(gamepad1.right_trigger);
        //vertical slide
        attachment.verticalSlide(gamepad1.left_trigger, gamepad1.left_bumper);
        //transfer wrist
        attachment.transferWrist(gamepad1.circle);
        //hook
        attachment.hook(gamepad1.dpad_up);
        //percise mode
        percise = attachment.percise(gamepad1.square);
       //manual slide override
        attachment.verticalOverride(gamepad1.cross, gamepad1.dpad_up, gamepad1.dpad_down);

        //field centric robot control
        if(gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0 || gamepad1.right_stick_x != 0 || gamepad1.right_bumper){
            driveTrain.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.options, percise, currentPose);
        } else{
            driveTrain.drive(0,0,0, false, percise, currentPose);
        }

        //update runing actions
        attachment.updateAction();
    }
}