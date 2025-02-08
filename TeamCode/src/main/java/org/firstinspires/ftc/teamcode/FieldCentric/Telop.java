package org.firstinspires.ftc.teamcode.FieldCentric;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Configuration.secondRobot.ConfigurationSecondRobot;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalGrabberRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalRollRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalSlideRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalWristRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalHangerRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalSlideRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalWristRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalGrabberRR;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.secondrobot.constants.ConfigConstants;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "FieldCentric")
public class Telop extends OpMode {
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();
    VerticalSlideRR verticalSlide; VerticalWristRR verticalWrist; VerticalGrabberRR verticalGrabber;
    VerticalHangerRR verticalHanger; HorizontalSlideRR horizontalSlide;
    HorizontalRollRR horizontalRoll; HorizontalGrabberRR horizontalGrabber;
    HorizontalWristRR horizontalWrist;
    DriveTrain driveTrain;

    @Override
    public void init() {
        //set up Pinpoint and Pose2d class
        Pose2d pose;
        PinpointDrive drive;

        //all of these class is under Configuration.secondRobot
         verticalSlide = new VerticalSlideRR(hardwareMap);
         verticalWrist = new VerticalWristRR(hardwareMap);
         verticalGrabber = new VerticalGrabberRR(hardwareMap);

         horizontalSlide = new HorizontalSlideRR(hardwareMap);
         horizontalRoll = new HorizontalRollRR(hardwareMap);
         horizontalGrabber = new HorizontalGrabberRR(hardwareMap);
         horizontalWrist = new HorizontalWristRR(hardwareMap);
         verticalHanger = new VerticalHangerRR(hardwareMap);


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
    //time
    double startTime = 0; double currTime = 0;
    double prevTimeRB = 0; double loopTimeRB = 0.0; boolean wasRB = false;
    double prevTimeTriangle = 0.0; double loopTimeTriangle = 0.0; boolean wasTriangle = false;
    double prevTimeRT = 0.0; double loopTimeRT = 0.0; boolean wasRT;

    //how long is the delay between press
    double desiredLoopms = 250.0;
    @Override
    public  void loop(){
        TelemetryPacket packet = new TelemetryPacket();
        //vertical grabber
        currTime = System.currentTimeMillis();
        loopTimeRB = currTime - prevTimeRB;
        //update need from gamepads
        if(gamepad1.right_bumper) {
            if (loopTimeRB >= desiredLoopms) {
                if (!wasRB) {
                    runningActions.add(new SequentialAction(
                            new InstantAction(() -> verticalGrabber.setPose(ConfigurationSecondRobot.verticalOpen))
                    ));
                } else {
                    if (wasRB) {
                        runningActions.add(new SequentialAction(
                                new InstantAction(() -> verticalGrabber.setPose(ConfigurationSecondRobot.verticalClose))
                        ));
                    }
                }
                prevTimeRB = currTime;
                wasRB = !wasRB;
            }
        }
        //horizontal slide
        loopTimeTriangle = currTime - prevTimeTriangle;
        //update need from gamepads
        if(gamepad1.triangle){
            if (loopTimeTriangle >= desiredLoopms){
                if (!wasTriangle){
                    runningActions.add(new SequentialAction(
                            new InstantAction(() -> horizontalSlide.setPose(ConfigurationSecondRobot.horizontalSlideExtend)),
                            new InstantAction(() -> horizontalWrist.setPose(ConfigurationSecondRobot.horizontalWristIntake)),
                            new InstantAction(() -> horizontalRoll.setPose(ConfigurationSecondRobot.slant))
                    ));
                } else {
                    if (wasTriangle){
                        runningActions.add(new SequentialAction(
                                new InstantAction(() -> horizontalSlide.setPose(ConfigurationSecondRobot.horizontalSlideRetract)),
                                new InstantAction(() -> horizontalWrist.setPose(ConfigurationSecondRobot.horizontalWristTransfer)),
                                new InstantAction(() -> horizontalRoll.setPose(ConfigurationSecondRobot.flat))
                        ));
                    }
                }
                prevTimeTriangle = currTime;
                wasTriangle = !wasTriangle;
            }
        }
        //horizontal grabber
        loopTimeRT = currTime - prevTimeTriangle;
        //update input from gamepad
        if(gamepad1.right_trigger != 0){
            if (loopTimeRT >= desiredLoopms){
                if (!wasRT){
                    runningActions.add(new SequentialAction(
                            new InstantAction(() -> horizontalGrabber.setPose(ConfigurationSecondRobot.horizontalGrabberClose))
                    ));
                } else{
                    if (wasRT){
                        runningActions.add(new SequentialAction(
                                new InstantAction(() -> horizontalGrabber.setPose(ConfigurationSecondRobot.horizontalGrabberOpen))
                        ));
                    }
                }
                prevTimeRT = currTime;
                wasRT = !wasRT;
            }
        }
        //field centric robot control
        if(gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0 || gamepad1.right_stick_x != 0 || gamepad1.right_bumper){
            driveTrain.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.left_bumper);
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
