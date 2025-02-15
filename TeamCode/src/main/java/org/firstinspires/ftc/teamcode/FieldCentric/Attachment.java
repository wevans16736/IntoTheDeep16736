package org.firstinspires.ftc.teamcode.FieldCentric;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

import org.firstinspires.ftc.teamcode.Configuration.secondRobot.ConfigurationSecondRobot;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalGrabberRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalRollRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalSlideRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.HorizontalWristRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalGrabberRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalHangerRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalSlideRR;
import org.firstinspires.ftc.teamcode.Configuration.secondRobot.VerticalWristRR;

import java.util.ArrayList;
import java.util.List;

public class Attachment {
    VerticalSlideRR verticalSlide; VerticalWristRR verticalWrist;
    VerticalGrabberRR verticalGrabber; HorizontalSlideRR horizontalSlide;
    HorizontalRollRR horizontalRoll; HorizontalGrabberRR horizontalGrabber;
    HorizontalWristRR horizontalWrist; VerticalHangerRR verticalHanger;
    double currTime; double desiredLoopms = 250;
    private List<Action> runningActions; private FtcDashboard dash;
    TelemetryPacket packet = new TelemetryPacket();

    public Attachment(VerticalSlideRR verticalSlide, VerticalWristRR verticalWrist,
                      VerticalGrabberRR verticalGrabber, HorizontalSlideRR horizontalSlide,
                      HorizontalRollRR horizontalRoll, HorizontalGrabberRR horizontalGrabber,
                      HorizontalWristRR horizontalWrist, VerticalHangerRR verticalHanger,
                      List<Action> runningActions, FtcDashboard dash){
        this.verticalSlide = verticalSlide;
        this.verticalWrist = verticalWrist;
        this.verticalGrabber = verticalGrabber;
        this.horizontalSlide = horizontalSlide;
        this.horizontalRoll = horizontalRoll;
        this.horizontalGrabber = horizontalGrabber;
        this.horizontalWrist = horizontalWrist;
        this.verticalHanger = verticalHanger;

        this.runningActions = runningActions;
        this.dash = dash;
        currTime = System.currentTimeMillis();
    }
    boolean wasRB = false; double prevTimeRB = 0.0; double LoopTimeRB;
    public void verticalGrabber(boolean right_bumper) {
        //vertical grabber
        LoopTimeRB= currTime - prevTimeRB;
        //update need from gamepads
        if(right_bumper){
            if (LoopTimeRB >= desiredLoopms) {
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
    }
    boolean wasTriangle = false; double prevTimeTriangle = 0.0; double loopTimeTriangle;
    public void horizontalSlide(boolean triangle) {
        loopTimeTriangle = currTime - prevTimeTriangle;
        if(triangle) {
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
                                new SleepAction(ConfigurationSecondRobot.horizontalSlideTime / 1000),
                                new InstantAction(() -> horizontalWrist.setPose(ConfigurationSecondRobot.horizontalWristTransfer)),
                                new SleepAction(.5),
                                new InstantAction(() -> verticalGrabber.setPose(ConfigurationSecondRobot.verticalClose)),
                                new SleepAction(ConfigurationSecondRobot.verticalCloseTime / 1000),
                                new InstantAction(() -> horizontalGrabber.setPose(ConfigurationSecondRobot.horizontalGrabberOpen))
                        ));
                    }
                }
                prevTimeTriangle = currTime;
                wasTriangle = !wasTriangle;
            }
        }
    }

    double loopTimeRT; boolean wasRT = false; double prevTimeRT = 0.0;
    public void horizontalGrabber(double right_trigger) {
        loopTimeRT = currTime - prevTimeRT;
        //update input from gamepad
        if (right_trigger >= .5) {
            if (loopTimeRT >= desiredLoopms) {
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
    }
    double wasLeft; double loopTimeLeft; double prevTimeLeft = 0.0;
    public void verticalSlide(double left_trigger, boolean left_bumper){
        //vertical preset
        loopTimeLeft = currTime - prevTimeLeft;
        //update input from gamepad
        if (left_trigger >= .5 || left_bumper) {
                if (loopTimeLeft >= desiredLoopms) {
                    if (left_trigger >= .5) {
                        wasLeft -= 1;
                        if (wasLeft <= 0) {
                            wasLeft = 0;
                        }
                    }
                    if (left_bumper) {
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
                    if (wasLeft == 1) {
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
    }
    double loopTimeCircle; double prevTimeCircle = 0.0; boolean wasCircle = false;
    public void transferWrist(boolean circle){
        //transfer wrist
        loopTimeCircle = currTime - prevTimeCircle;
        //update input from gamepad
        if (circle) {
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
    }
    double loopTimeDPU; double prevTimeDPU = 0.0; boolean wasDPU;
    public void hook(boolean dpad_up){
        //hook
        loopTimeDPU = currTime - prevTimeDPU;
        //update input from gamepad
        if (dpad_up) {
            if (!wasCross) {
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
        }
    }
    double loopTimeSquare; double prevTimeSquare = 0.0; boolean wasSquare;
    public boolean percise(boolean square){
        loopTimeSquare = currTime - prevTimeSquare;
        //update input from gamepad
        if (square) {
            if (loopTimeSquare >= desiredLoopms) {
                prevTimeSquare = currTime;
                wasSquare = !wasSquare;
            }
        }
        return wasSquare;
    }
    double loopTimeCross; double prevTimeCross = 0.0; boolean wasCross = false; int slidePose;
    double loopTimedpad; double prevTimedpad = 0.0;
    public void verticalOverride(boolean cross, boolean dpad_up, boolean dpad_down){
        //manual override for vertical slide
        loopTimeCross = currTime - prevTimeCross;
        loopTimedpad = currTime - prevTimedpad;
        //update input from gamepad
        if (cross) {
            if(loopTimeCross >= desiredLoopms){
                if(wasCross){
                    slidePose = verticalSlide.returnPose();
                   if(loopTimedpad >= desiredLoopms){
                       if(dpad_up){
                           slidePose += 50;
                           verticalSlide.setPose(slidePose);
                       }
                       if(dpad_down){
                           slidePose -= 50;
                           verticalSlide.setPose(slidePose);
                       }
                       prevTimedpad = currTime;
                   }
                }
                prevTimeCross = currTime;
                wasCross = !wasCross;
            }
        }
    }
    public void updateAction(){
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
