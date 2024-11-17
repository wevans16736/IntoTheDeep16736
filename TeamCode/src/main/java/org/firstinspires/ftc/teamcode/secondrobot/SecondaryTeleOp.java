package org.firstinspires.ftc.teamcode.secondrobot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.secondrobot.ArmActions;

@TeleOp(name = "Tele Op second robot", group = "Linear Opmode")
public class SecondaryTeleOp extends LinearOpMode {

    private ArmActions arm = null;
    double liftSpdMult = 0.8 ;

    @Override
    public void runOpMode() {

        arm = new ArmActions(hardwareMap, telemetry);


        //Set Speed for teleOp. Mecannum wheel speed.
        //driveActions.setSpeed(1.0);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        while (opModeIsActive()) {
            arm.teleOpArm(gamepad2.left_stick_y, 0.5);
            telemetry.update();
        }
    }
}
