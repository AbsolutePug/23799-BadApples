package org.firstinspires.ftc.teamcode.Core;

import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;
/**
 * Extends functionality of RobotHardware for scheduled actions during autonomous
 */
public class AutoCore extends RobotHardware {
    private boolean actionStopped = false;

    private void cancelAction() {
        actionStopped = true;
    }

    private double easeOutQuad(double x) {
        return 1 - (1 - x) * (1 - x);
    } // This should probably be moved somewhere else but its not necessary right now

/**
 * Move within an x, y range for a given time
 * @param x speed to move left/right relative to the ROBOT's position
 * @param y speed to move forward relative to the ROBOT's position
 * @param time (milliseconds)
 */
    public void move(double x, double y, double time) {
        actionStopped = false;
        ElapsedTime timeout = new ElapsedTime();
        while (timeout.milliseconds() < time && !actionStopped) {
            double speed = easeOutQuad(timeout.milliseconds()/time);
            speed = Math.min(speed,1); // Normalize

            double front_left   = (-y-x)*speed;
            double front_right  = (-y+x)*speed;
            double back_left    = (-y+x)*speed;
            double back_right   = (-y-x)*speed;

            setPowers(front_left,front_right,back_left,back_right);
        }
        setPowers(0,0,0,0);
    }
    public void shoot() {
        actionStopped = false;
        ElapsedTime timeout = new ElapsedTime();
        while (timeout.milliseconds() < 10000 && !actionStopped) {
            setLauncher(true);
        }
        setLauncher(false);
    }
    public void turn(double target_angle) {
        actionStopped = false;
        resetHeading();
        double tolerance = 5;
        double target_a = target_angle - tolerance;
        double target_b = target_angle + tolerance;

        while (getHeading() <= Math.min(target_a, target_b) || getHeading() >= Math.max(target_a, target_b)) { // pick whichever target bound is the lower and whichever target is the higher to compare
            if (getHeading() < target_angle) {
                setPowers(-0.2,0.2,-0.2,0.2);
            } else {
                setPowers(0.2,-0.2,0.2,-0.2);
            }
            if (actionStopped) break;
        }
    }
}