package org.firstinspires.ftc.teamcode.BadApples.Core;

import com.qualcomm.robotcore.util.ElapsedTime;
/**
 * Extends functionality of RobotHardware for scheduled actions during autonomous
 */
public class AutoCore extends RobotFunctionCore {
    public enum TeamColor {RED, BLUE}
    private boolean actionStopped = false;

    private void cancelAction() {
        actionStopped = true;
    }

    private double easeOutQuad(double x) {
        return Math.min(1 - (1 - x) * (1 - x),1);
    } // This should probably be moved somewhere else but its not necessary right now

/**
 * Move within an x, y range for a given time
 * @param speed_laterally speed to move left/right relative to the ROBOT's position
 * @param speed_axially speed to move forward relative to the ROBOT's position
 * @param time (milliseconds)
 */
    public void move(double speed_laterally, double speed_axially, double time) {
        actionStopped = false;
        ElapsedTime timeout = new ElapsedTime();
        while (timeout.milliseconds() < time && !actionStopped) {
            double speed = easeOutQuad(timeout.milliseconds()/time);
            speed = Math.min(speed,1); // Normalize

            double front_left   = (-speed_axially-speed_laterally)*speed;
            double front_right  = (-speed_axially+speed_laterally)*speed;
            double back_left    = (-speed_axially+speed_laterally)*speed;
            double back_right   = (-speed_axially-speed_laterally)*speed;

            setPowers(front_left,front_right,back_left,back_right);
        }
        setPowers(0,0,0,0);
    }
    public void shoot() {
        actionStopped = false;
        ElapsedTime timeout = new ElapsedTime();
        while (timeout.milliseconds() < 3000 && !actionStopped) {
            setLauncher(true,false);
        }
        timeout.reset();
        while (timeout.milliseconds() < 1000) {
            setLauncher(false,false);
        }
        timeout.reset();
        while (timeout.milliseconds() < 3000 && !actionStopped) {
            setLauncher(false,true);
        }
        setLauncher(false);
    }
    public void turn(double target_angle) {
        actionStopped = false;
        resetHeading();
        double tolerance = 3;
        double target_a = target_angle - tolerance;
        double target_b = target_angle + tolerance;

        while (getHeading() <= Math.min(target_a, target_b) || getHeading() >= Math.max(target_a, target_b)) { // pick whichever target bound is the lower and whichever target is the higher to compare to check if outside desired value

            // The desired speed = difference between the target and current heading
            double speed = Math.abs((target_angle - getHeading())/10);
            speed = Math.min(speed, 1);
            speed = Math.max(speed, 0.1);
            if (getHeading() < target_angle) {
                setPowers(-0.3*speed,0.3*speed,-0.3*speed,0.3*speed);
            } else {
                setPowers(0.3*speed,-0.3*speed,0.3*speed,-0.3*speed);
            }
            if (actionStopped) break;
        }
    }
    public void waitUntilFlywheelIsReady() {
        actionStopped = false;
        while (!getFlywheelReady() && !actionStopped) {
            // this is bad but idc right now if it works
            ElapsedTime timeout = new ElapsedTime();
            while (timeout.milliseconds() < 100 && !actionStopped) {

            }
        }
    }
}