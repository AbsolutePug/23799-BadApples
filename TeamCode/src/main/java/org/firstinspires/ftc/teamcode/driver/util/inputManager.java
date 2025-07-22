package org.firstinspires.ftc.teamcode.driver.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class inputManager extends LinearOpMode {

    public inputManager () {} // unfinished

    @Override
    public void runOpMode() throws InterruptedException {
        // idk
    }

    private boolean gamepad_a_last = false;
    boolean gamepad_right_bumper_last = false;
    boolean gamepad_left_bumper_last = false;

    public boolean aPressed() {
        return (gamepad1.a && !gamepad_a_last);
    }
    public boolean aDown() {
        return gamepad1.a;
    }
    public boolean rightBumperPressed() {
        return (gamepad1.right_bumper && !gamepad_right_bumper_last);
    }
    public boolean leftBumperPressed() {
        return (gamepad1.left_bumper && !gamepad_left_bumper_last);
    }

    public void updateInputs() {
        gamepad_a_last = gamepad1.a;
        gamepad_left_bumper_last = gamepad2.left_bumper;
        gamepad_right_bumper_last = gamepad2.right_bumper;
    }
}
