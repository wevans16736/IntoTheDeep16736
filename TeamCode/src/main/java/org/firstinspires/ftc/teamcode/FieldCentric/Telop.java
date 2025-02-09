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

import org.firstinspires.ftc.teamcode.Configuration.secondRobot.ConfigurationSecondRobot;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalGrabberRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalRollRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalSlideRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalWristRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalHangerRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalSlideRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalWristRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalGrabberRR;
import org.firstinspires.ftc.teamcode.GlobalVarable;
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
        if(!GlobalVarable.autoStarted){
            this.currentPose = new Pose2d(0,0,0);
        } else{
            this.currentPose = GlobalVarable.currentPose;
        }
        GlobalVarable.autoStarted = false;
    }

    //time
    double startTime = 0;
    double currTime = 0;
    double prevTimeRB = 0.0;
    double loopTimeRB = 0.0;
    boolean wasRB = false;
    double prevTimeTriangle = 0.0;
    double loopTimeTriangle = 0.0;
    boolean wasTriangle = false;
    double prevTimeRT = 0.0;
    double loopTimeRT = 0.0;
    boolean wasRT;
    double prevTimeLeft = 0.0;
    double loopTimeLeft = 0.0;
    int wasLeft = 0;
    double prevTimeCircle = 0.0;
    double loopTimeCircle = 0.0;
    boolean wasCircle = false;
    double prevTimeDPU = 0.0;
    double loopTimeDPU = 0.0;
    boolean wasDPU = false;
    double prevTimeSquare = 0.0;
    double loopTimeSquare = 0.0;
    boolean wasSquare = false;
    double prevTimeCross = 0.0;
    double loopTimeCross = 0.0;
    boolean wasCross = false;
    int slidePose = 0;
    double percise = 1;
    //how long is the delay between press
    double desiredLoopms = 250.0;
    double RTms = 500;

    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();
        //vertical grabber
        currTime = System.currentTimeMillis();
        loopTimeRB = currTime - prevTimeRB;
        //update need from gamepads
        if (gamepad1.right_bumper) {
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
        if (gamepad1.triangle) {
            if (loopTimeTriangle >= desiredLoopms) {
                if (!wasTriangle) {
                    runningActions.add(new SequentialAction(
                            new InstantAction(() -> horizontalSlide.setPose(ConfigurationSecondRobot.horizontalSlideExtend)),
                            new SleepAction(.5),
                            new InstantAction(() -> horizontalWrist.setPose(ConfigurationSecondRobot.horizontalWristIntake)),
                            new InstantAction(() -> horizontalRoll.setPose(ConfigurationSecondRobot.slant)),
                            new InstantAction(() -> verticalGrabber.setPose(ConfigurationSecondRobot.verticalOpen)),
                            new InstantAction(() -> horizontalGrabber.setPose(ConfigurationSecondRobot.horizontalGrabberOpen))
                    ));
                } else {
                    if (wasTriangle) {
                        runningActions.add(new SequentialAction(
                                new InstantAction(() -> horizontalWrist.setPose(ConfigurationSecondRobot.horizontalWristHover)),
                                new InstantAction(() -> horizontalRoll.setPose(ConfigurationSecondRobot.flat)),
                                new InstantAction(() -> horizontalSlide.setPose(ConfigurationSecondRobot.horizontalSlideRetract)),
                                new InstantAction(() -> verticalGrabber.setPose(ConfigurationSecondRobot.verticalOpen)),
                                new InstantAction(() -> verticalWrist.setPose(ConfigurationSecondRobot.verticalWristIntake)),
                                new SleepAction(ConfigurationSecondRobot.horizontalSlideTime/1000),
                                new InstantAction(() -> horizontalWrist.setPose(ConfigurationSecondRobot.horizontalWristTransfer)),
                                new SleepAction(.5),
                                new InstantAction(() -> verticalGrabber.setPose(ConfigurationSecondRobot.verticalClose)),
                                new SleepAction(ConfigurationSecondRobot.verticalCloseTime /1000),
                                new InstantAction(() -> horizontalGrabber.setPose(ConfigurationSecondRobot.horizontalGrabberOpen))
                        ));
                    }
                }
                prevTimeTriangle = currTime;
                wasTriangle = !wasTriangle;
            }
        }
        //horizontal grabber
        loopTimeRT = currTime - prevTimeRT;
        //update input from gamepad
        if (gamepad1.right_trigger >= .5) {
            if (loopTimeRT >= RTms) {
                if (!wasRT) {
                    runningActions.add(new SequentialAction(
                            new InstantAction(() -> horizontalGrabber.setPose(ConfigurationSecondRobot.horizontalGrabberClose))
                    ));
                } else {
                    if (wasRT) {
                        runningActions.add(new SequentialAction(
                                new InstantAction(() -> horizontalGrabber.setPose(ConfigurationSecondRobot.horizontalGrabberOpen))
                        ));
                    }
                }
                prevTimeRT = currTime;
                wasRT = !wasRT;
            }
        }
        //vertical preset
        loopTimeLeft = currTime - prevTimeLeft;
        //update input from gamepad
        if (gamepad1.left_trigger >= .5 || gamepad1.left_bumper) {
            if (loopTimeLeft >= desiredLoopms) {
                if (gamepad1.left_trigger >= .5) {
                    wasLeft -= 1;
                    if (wasLeft <= 0) {
                        wasLeft = 0;
                    }
                }
                if (gamepad1.left_bumper) {
                    wasLeft += 1;
                    if (wasLeft >= 3) {
                        wasLeft = 3;
                    }
                }


            if (wasLeft == 0) {
                runningActions.add(new SequentialAction(
                        new InstantAction(() -> verticalSlide.setPose(ConfigurationSecondRobot.bottom)),
                        new InstantAction(() -> verticalWrist.setPose(ConfigurationSecondRobot.verticalWristIntake)),
                        new InstantAction(() -> verticalGrabber.setPose(ConfigurationSecondRobot.verticalOpen))
                ));
            }
            if(wasLeft == 1){
                runningActions.add(new SequentialAction(
                        new InstantAction(() -> verticalWrist.setPose(ConfigurationSecondRobot.verticalWristWall)),
                        new InstantAction(() -> verticalSlide.setPose(ConfigurationSecondRobot.bottom)),
                        new InstantAction(() -> verticalGrabber.setPose(ConfigurationSecondRobot.verticalClose))
                ));
            }
            if (wasLeft == 2) {
                runningActions.add(new SequentialAction(
                        new InstantAction(() -> verticalSlide.setPose(ConfigurationSecondRobot.highBar)),
                        new InstantAction(() -> verticalWrist.setPose(ConfigurationSecondRobot.verticalWristBar))
                ));
            }
            if (wasLeft == 3) {
                runningActions.add(new SequentialAction(
                        new InstantAction(() -> verticalWrist.setPose(ConfigurationSecondRobot.verticalWristBasket)),
                        new InstantAction(() -> verticalSlide.setPose(ConfigurationSecondRobot.topBasket))
                ));
            }
            prevTimeLeft = currTime;
            }
        }

        //transfer wrist
        loopTimeCircle = currTime - prevTimeCircle;
        //update input from gamepad
        if (gamepad1.circle) {
            if (loopTimeCircle <= desiredLoopms) {
                if (wasCircle) {
                    runningActions.add(new SequentialAction(
                            new InstantAction(() -> verticalWrist.setPose(ConfigurationSecondRobot.verticalWristIntake))
                    ));
                } else {
                    if (!wasCircle) {
                        runningActions.add(new SequentialAction(
                                new InstantAction(() -> verticalWrist.setPose(ConfigurationSecondRobot.verticalWristWall)),
                                new InstantAction(() -> verticalGrabber.setPose(ConfigurationSecondRobot.verticalOpen))
                        ));
                    }
                }
                prevTimeCircle = currTime;
                wasCircle = !wasCircle;
            }
        }
        //hook
        loopTimeDPU = currTime - prevTimeDPU;
        //update input from gamepad
        if (gamepad1.dpad_up) {
            if (loopTimeDPU >= desiredLoopms) {
                if (wasDPU) {
                    runningActions.add(new SequentialAction(
                            new InstantAction(() -> verticalHanger.setPose(ConfigurationSecondRobot.verticalHangIn))
                    ));
                } else {
                    if (!wasDPU) {
                        runningActions.add(new SequentialAction(
                                new InstantAction(() -> verticalHanger.setPose(ConfigurationSecondRobot.verticalHangOut))
                        ));
                    }
                }
                prevTimeDPU = currTime;
                wasDPU = !wasDPU;
            }
        }
        //percise mode
        loopTimeSquare = currTime - prevTimeSquare;
        //update input from gamepad
        if (gamepad1.square) {
            if (loopTimeSquare >= desiredLoopms) {
                prevTimeSquare = currTime;
                wasSquare = !wasSquare;
            }
        }
        //manual override for vertical slide
        loopTimeCross = currTime - prevTimeCross;
        //update input from gamepad
        if (gamepad1.cross) {
           if(loopTimeCross >= desiredLoopms){
               if(wasCross){
                   slidePose = verticalSlide.returnPose();
                   if (gamepad1.left_stick_y != 0) {
                       slidePose -= (int) (gamepad1.left_stick_y * 5);
                       runningActions.add(new SequentialAction(
                               new InstantAction(() -> verticalSlide.setPose(slidePose))
                       ));
                   }
               }
               prevTimeCross = currTime;
               wasCross = !wasCross;
           }
        }

        //field centric robot control
        if(gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0 || gamepad1.right_stick_x != 0 || gamepad1.right_bumper){
            driveTrain.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.options, wasSquare, currentPose);
        } else{
            driveTrain.drive(0,0,0, false, wasSquare, currentPose);
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