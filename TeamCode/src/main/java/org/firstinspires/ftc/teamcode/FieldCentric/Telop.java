package org.firstinspires.ftc.teamcode.FieldCentric;

import android.provider.Settings;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.sun.source.tree.DoWhileLoopTree;

import org.firstinspires.ftc.teamcode.Configuration.secondRobot.ConfigurationSecondRobot;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalGrabberRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalRollRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalSlideRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalWristRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalGrabberRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalHangerRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalSlideRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalWristRR;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.secondrobot.constants.ConfigConstants;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "FieldCentric")
public class Telop extends OpMode {
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();
    Servo VG;
    DriveTrain driveTrain;

    @Override
    public void init() {
        //set up Pinpoint and Pose2d class
        Pose2d pose;
        PinpointDrive drive;

        //all of these class is under Configuration.secondRobot
        VerticalSlideRR verticalSlide = new VerticalSlideRR(hardwareMap);
        VerticalWristRR verticalWrist = new VerticalWristRR(hardwareMap);
//        VerticalGrabberRR verticalGrabber = new VerticalGrabberRR(hardwareMap);

        HorizontalSlideRR horizontalSlide = new HorizontalSlideRR(hardwareMap);
        HorizontalRollRR horizontalRoll = new HorizontalRollRR(hardwareMap);
        HorizontalGrabberRR horizontalGrabber = new HorizontalGrabberRR(hardwareMap);
        HorizontalWristRR horizontalWrist = new HorizontalWristRR(hardwareMap);
        VerticalHangerRR verticalHanger = new VerticalHangerRR(hardwareMap);

        VG = hardwareMap.get(Servo.class, ConfigConstants.VERTICAL_GRABBER);
        VG.setPosition(.55);

        //todo localization after auto?
        pose = new Pose2d(0, 0, Math.toRadians(90));
        drive = new PinpointDrive(hardwareMap, pose);

        //setup the drive train
        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, ConfigConstants.FRONT_LEFT);
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, ConfigConstants.FRONT_RIGHT);
        DcMotorEx rearLeft = hardwareMap.get(DcMotorEx.class, ConfigConstants.BACK_LEFT);
        DcMotorEx rearRight = hardwareMap.get(DcMotorEx.class, ConfigConstants.BACK_RIGHT);
        IMU imu = hardwareMap.get(IMU.class, ConfigConstants.IMU);
        driveTrain = new DriveTrain(frontLeft, frontRight, rearLeft, rearRight, imu);
    }
    double startTime = 0; double currTime = 0; double prevTime = 0; double loopTime = 0.0;
    double desiredLoopms = 250.0;

    boolean wasX = false;

    @Override
    public  void loop(){
        TelemetryPacket packet = new TelemetryPacket();
        //vertical grabber
        currTime = System.currentTimeMillis();
        loopTime = currTime - prevTime;
        //update need from gamepads
        if(gamepad1.x) {
            if (loopTime >= desiredLoopms) {
                if (!wasX) {
                    runningActions.add(new SequentialAction(
                            new InstantAction(() -> VG.setPosition(.55))
                    ));
                } else {
                    if (wasX) {
                        runningActions.add(new SequentialAction(
                                new InstantAction(() -> VG.setPosition(0.41))
                        ));
                    }
                }
                prevTime = currTime;
                wasX = !wasX;
            }
        }
        //field centric robot control
        if(gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0 || gamepad1.right_stick_x != 0 || gamepad1.right_bumper){
            driveTrain.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.right_bumper);
        } else{
            driveTrain.drive(0,0,0, false);
        }


        //update runing actions
        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        runningActions = newActions;

        dash.sendTelemetryPacket(packet);
    }
}
