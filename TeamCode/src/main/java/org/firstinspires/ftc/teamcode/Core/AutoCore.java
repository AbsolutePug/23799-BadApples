package org.firstinspires.ftc.teamcode.Core;

import com.qualcomm.robotcore.util.ElapsedTime;
/**
 * Extends functionality of RobotHardware for scheduled actions during autonomous
 */
public class AutoCore extends RobotHardware {
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
        ElapsedTime timeout = new ElapsedTime();
        while (timeout.milliseconds() < time) {
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
}


// FL: axial - lateral - yaw)
// FR: axial + lateral + yaw)
// BL: axial + lateral - yaw)
// BR: axial - lateral + yaw)
