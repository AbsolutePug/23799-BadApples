package org.firstinspires.ftc.teamcode.Core;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Written by <a href="https://github.com/AbsolutePug">Robert Maddox (AbsolutePug)</a> 2025
 * <p>
 * Stores a few functions to make more sophisticated Telemetry displaying easier / more abstract
 */
public class telemetrySophisticated {
    Telemetry telemetry;
    public telemetrySophisticated(Telemetry driverStation) {
        telemetry = driverStation;
    }

    private double bar_width = 20; // The width of the progress bar

    public void addBoolean(String caption, boolean value) {
        if (value) {
            telemetry.addData(caption,"██████████");
        } else {
            telemetry.addData(caption,"░░░░░░░░░░");
        }
    }
    public void addPercent(String caption, double value) {
        double rounded = Math.min(Math.round(Math.abs(value)*bar_width), bar_width); // Turn the value from a decimal to a number 1-10 and round it
        String result = "";

        for (int i = 0; i < rounded; i ++) { // Create the filled in bars
            result += "█";
        }
        for (int j = 0; j < bar_width-rounded; j++) { // Fill the rest with unfilled bars
            result += "░";
        }

        telemetry.addData(caption, result);
    }
}
