package org.firstinspires.ftc.teamcode.secondrobot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Configuration.secondRobot.ConfigurationSecondRobot;
import org.firstinspires.ftc.teamcode.secondrobot.horizontalslide.HorizontalIntakeActions;
import org.firstinspires.ftc.teamcode.secondrobot.horizontalslide.HorizontalIRollActions;
import org.firstinspires.ftc.teamcode.secondrobot.horizontalslide.HorizontalSlideActions;
import org.firstinspires.ftc.teamcode.secondrobot.horizontalslide.HorizontalWristActions;
import org.firstinspires.ftc.teamcode.secondrobot.verticalslide.VerticalGrabberActions;
import org.firstinspires.ftc.teamcode.secondrobot.verticalslide.VerticalSlideActions;
import org.firstinspires.ftc.teamcode.secondrobot.verticalslide.VerticalWristActions;
import org.opencv.core.Point;

public abstract class HelperActions extends LinearOpMode {
    public final double SPEED = 0.5;

    public static int LEFT = 1;
    public static int RIGHT = 2;
    public static int FORWARDS = 3;
    public static int BACKWARDS = 4;
    public static int LOW = 5;
    public static int MEDIUM = 6;
    public static int HIGH = 7;

    private int speeding = 0;
    private double speed = 0.6;

    /**drive speed configuration**/
    public double getSpeed() { return speed; }
    public void setSpeed(double speed) {
        this.speed = speed;
    }
    public void changeSpeed(DriveActions driveActions, boolean upOne, boolean downOne, boolean upTwo, boolean downTwo, double scalarMultiple) {
        if (upOne) {
            speeding++;
            if (speeding == 1) {
                speed = speed + 0.1;
            }
        }
        if (downOne) {
            speeding++;
            if (speeding == 1) {
                speed = speed - 0.1;
            }
        }
        if (upTwo) {
            speeding++;
            if (speeding == 1) {
                speed = speed + 0.2;
            }
        }
        if (downTwo) {
            speeding++;
            if (speeding == 1) {
                speed = speed - 0.2;
            }
        }
        if (!upOne && !downOne && !upTwo && !downTwo) {
            speeding = 0;
        }
        if (speed < 0) {
            speed = 0;
        }
        if (speed > 1.0) {
            speed = 1.0;
        }
        double speedLeft = 1.0 - speed;
        driveActions.setSpeed(speed + (scalarMultiple * speedLeft));
        telemetry.addData("speed: ", speed);
    }

    double prevSpeed;
    boolean low = false;
    boolean prevToggle = false;
    double lowSpeed = 0.35;
    public void toggleSpeed(boolean toggle) {
        if (toggle && !prevToggle) {
            low = !low;
            if (low) {
                prevSpeed = speed;
                speed = lowSpeed;
            } else {
                lowSpeed = speed;
                speed = prevSpeed;
            }
        }
         prevToggle = toggle;
    }

    public boolean reverseSpeed = false;
    public void setReverseSpeed(boolean toggle) {
        reverseSpeed = toggle;
    }
    public int getReverseSpeed() {
        if (reverseSpeed) {
            return -1;
        } else {
            return 1;
        }
    }

    public void moveToBlock(DetectBlockActions detectBlockActions, DriveActions driveActions, HorizontalSlideActions horizontalSlideActions, HorizontalIRollActions horizontalIRollActions) {
        Point position = detectBlockActions.pixelToPosition();
        driveActions.drive(0, position.x, 0);
        horizontalSlideActions.teleOpArmMotor(position.y, 2);
        double angle = detectBlockActions.angle;
        while (angle > 180){
            angle -= 180;
        }
        horizontalIRollActions.setPosition(angle / 180);
    }

    public double adjustedHSlideSpeed(double inputSpeed) {
        double workingSpeed = Math.abs(inputSpeed);
        double startingSpeed = 0.1;
        double endingSpeed = 5;
        //0.1x = 5x + 9
        double transitionPoint = 4.0 / 4.9;
        if (workingSpeed < transitionPoint) {
            workingSpeed = workingSpeed * startingSpeed;
        } else {
            workingSpeed = workingSpeed * endingSpeed - 4;
        }
        return workingSpeed * Math.signum(inputSpeed);
    }
     /**slide configuration?**/
    boolean wasOverrideSlide = true;
    boolean overrideSlide = true;
    double overrideSlideThreshold = 5;
    public void updateExchangeAssembly(VerticalGrabberActions grabber, VerticalWristActions verticalWrist, HorizontalWristActions horizontalWrist, HorizontalSlideActions horizontalArm, VerticalSlideActions verticalSlide, HorizontalIRollActions horizontalIRoll, HorizontalIntakeActions intake) {
        //tells the horizontal intake to be open when the vertical grabber grabs
        intake.setIsVertGrabberClosed(grabber.isClose());

        //tells the vertical wrist when the slide is up
        verticalWrist.setSlideUp(verticalSlide.getSlidePosition() > ConfigurationSecondRobot.highBar + 700);
        verticalWrist.setSlideMiddle(verticalSlide.getSlidePosition() > ConfigurationSecondRobot.highBar - 700);

        //tells the horizontal slide to stop and let the horizontal wrist flip up or flip down when going in or out
        overrideSlide = horizontalArm.getSlidePosition() < overrideSlideThreshold && !horizontalWrist.override;
        horizontalWrist.setIsSlideIn(overrideSlide);
        overrideSlide(horizontalArm);
        wasOverrideSlide = overrideSlide;

        //tells the horizontal roll to go straight up when transfering from grabber to grabber
        horizontalIRoll.setTransfer(horizontalWrist.override || overrideSlide);
    }
    double startTime = 0;
    public void overrideSlide(HorizontalSlideActions horizontalArm) {
        if (overrideSlide && !wasOverrideSlide) {
            startTime = System.currentTimeMillis();
        }
        if (System.currentTimeMillis() < startTime + 420) {
            horizontalArm.setOverride(true);
        } else {
            horizontalArm.setOverride(false);
        }
    }

    public double verticalGrabberOpenStartTime = 0;
    public double verticalWristBackwardStartTime = 0;
    public boolean closeVerticalAssembly(VerticalGrabberActions verticalGrabber, VerticalWristActions verticalWrist, VerticalSlideActions verticalSlide){
        //Assume the vertical assembly is closed until proven otherwise
        boolean isClosed = true;
        //Set the vertical slide to go to the bottom
        verticalSlide.goToPreset(false, true, false, false);
        //if the distance between the vertical slide and the bottom is significant, the vertical assembly is not closed
        if (Math.abs(verticalSlide.getSlidePosition() - ConfigurationSecondRobot.bottom) > 5) {
            isClosed = false;
        }
        //if the vertical grabber is closed, open it and start a timer to run until it's open
        if (verticalGrabber.isClose()) {
            verticalGrabber.open();
            verticalGrabberOpenStartTime = System.currentTimeMillis();
        }
        //if the vertical grabber timer is not over, the vertical assembly is not closed
        if (System.currentTimeMillis() < verticalGrabberOpenStartTime + ConfigurationSecondRobot.verticalOpenTime) {
            isClosed = false;
        }
        //if the vertical wrist is flipped out, flip it back and start a timer to run until it's back
        if (verticalWrist.forward) {
            verticalWrist.backward();
            verticalWristBackwardStartTime = System.currentTimeMillis();
        }
        //if the vertical wrist timer is not over, the vertical assembly is not closed
        if (System.currentTimeMillis() < verticalWristBackwardStartTime + ConfigurationSecondRobot.verticalWristWalltoIntake) {
            isClosed = false;
        }
        return isClosed;
    }
    public double horizontalWristMiddleStartTime = 0;
    double timeTilWristIsUp = 0;
    double horizontalWristUpStartTime = 0;
    double horizontalRollTurnStartTime = 0;
    public boolean closeHorizontalAssembly(HorizontalWristActions horizontalWrist, HorizontalSlideActions horizontalSlide, HorizontalIntakeActions intake, HorizontalIRollActions horizontalRoll) {
        //Assume the horizontal assembly is closed until proven otherwise
        boolean isClosed = true;
        //If the Horizontal slide is out, move the slide in and move the wrist to be able to pass through the submersible walls
        if (horizontalSlide.getSlidePosition() > 10){
            isClosed = false;
            if (horizontalWrist.forward) {
                horizontalWrist.setForward(false);
                //Start a timer to go until the horizontal wrist is in the right position
                horizontalWristMiddleStartTime = System.currentTimeMillis();
            }
            //move the slide in, if the wrist is moving move the slide slower
            if (System.currentTimeMillis() < horizontalWristMiddleStartTime + ConfigurationSecondRobot.horizontalWristtoMiddleTime) {
                horizontalSlide.teleOpArmMotor(-0.5, 1);
            } else {
                horizontalSlide.setSlideDistance(0, 3000);
            }
        } else {
            //if the slide is in, move the wrist to the transfer and set a timer to wait until it's done
            if (!horizontalWrist.override) {
                horizontalWrist.setOverride(true);
                horizontalWristUpStartTime = System.currentTimeMillis();
                //if the wrist is down, set the timer to the time it'll take to move all the way up
                if (horizontalWrist.forward) {
                    timeTilWristIsUp = ConfigurationSecondRobot.horizontalWristDowntoUpTime;
                } else {
                    //If the wrist is middle, the time it needs is the time from middle to up. if it's moving from down to middle, the time it has been moving needs to be subtracted from the timer
                    if (System.currentTimeMillis() > horizontalWristMiddleStartTime + ConfigurationSecondRobot.horizontalWristtoMiddleTime) {
                        timeTilWristIsUp = ConfigurationSecondRobot.horizontalWristtoMiddleTime;
                    } else {
                        double timeWristHasBeenTravelling = Range.clip(System.currentTimeMillis() - horizontalWristMiddleStartTime, 0, ConfigurationSecondRobot.horizontalWristtoMiddleTime);
                        timeTilWristIsUp = ConfigurationSecondRobot.horizontalWristDowntoUpTime - timeWristHasBeenTravelling;
                    }
                }
            }
            if (System.currentTimeMillis() < horizontalWristUpStartTime + timeTilWristIsUp) {
                //if the timer says the wrist is not up yet, the horizontal assembly isn't closed
                isClosed = false;
            }
        }

        //close the horizontal roll and set a timer to wait until it's done
        if (!horizontalRoll.isFlat()) {
            //going false then true switches the flat state, which we know is not flat, switching it to flat
            horizontalRoll.teleOp(false);
            horizontalRoll.teleOp(true);
            horizontalRollTurnStartTime = System.currentTimeMillis();
        }
        if (System.currentTimeMillis() < horizontalRollTurnStartTime + ConfigurationSecondRobot.flatToSidewaysTime) {
            isClosed = false;
        }

        return isClosed;
    }
    double transferState = 0;
    double verticalGrabberCloseStartTime = 0;
    double horizontalIntakeOpenStartTime = 0;
    public boolean transferSample(VerticalGrabberActions verticalGrabber, HorizontalIntakeActions horizontalIntake) {
        if (transferState == 0) {
            verticalGrabber.close();
            verticalGrabberCloseStartTime = System.currentTimeMillis();
            transferState = 1;
        }
        if (transferState == 1 && System.currentTimeMillis() > verticalGrabberCloseStartTime + ConfigurationSecondRobot.verticalCloseTime) {
            horizontalIntake.open();
            horizontalIntakeOpenStartTime = System.currentTimeMillis();
            transferState = 2;
        }
        if (transferState == 2 && System.currentTimeMillis() > horizontalIntakeOpenStartTime + ConfigurationSecondRobot.horizontalGrabberOpen) {
            transferState = 3;
        }
        return (transferState == 3);
    }
    public void placeSample(VerticalGrabberActions verticalGrabber, VerticalWristActions verticalWrist, VerticalSlideActions verticalSlide, HorizontalWristActions horizontalWrist, HorizontalSlideActions horizontalSlide, HorizontalIntakeActions intake, HorizontalIRollActions horizontalRoll) {
        if (placeState == 0) {
            boolean closeVert = closeVerticalAssembly(verticalGrabber, verticalWrist, verticalSlide);
            boolean closeHorizontal = closeHorizontalAssembly(horizontalWrist, horizontalSlide, intake, horizontalRoll);
            if (closeVert && closeHorizontal) {
                placeState = 1;
            }
        } else if (placeState == 1) {
            transferState = 0;
            transferSample(verticalGrabber, intake);
            placeState = 2;
        } else if (placeState == 2) {
            if (transferSample(verticalGrabber, intake)) {
                placeState = 3;
            }
        } else if (placeState == 3) {
            verticalWrist.forward();
            verticalSlide.setSlidePosition(2300, 3000);
        }
        telemetry.addData("placeState", placeState);
    }
    double placeState = 0;
    public void resetPlaceState() {
        placeState = 0;
    }
    boolean wasActivatePlaceSample = false;
    public void managePlaceSample(boolean activate, VerticalGrabberActions verticalGrabber, VerticalWristActions verticalWrist, VerticalSlideActions verticalSlide, HorizontalWristActions horizontalWrist, HorizontalSlideActions horizontalSlide, HorizontalIntakeActions intake, HorizontalIRollActions horizontalRoll) {
        if (activate && !wasActivatePlaceSample) {
            resetPlaceState();
        }
        if(activate) {
            placeSample(verticalGrabber, verticalWrist, verticalSlide, horizontalWrist, horizontalSlide, intake, horizontalRoll);
        }
        wasActivatePlaceSample = activate;
    }
}