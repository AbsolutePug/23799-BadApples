package org.firstinspires.ftc.teamcode.badApples.core;

/**
 * Written by <a href="https://github.com/AbsolutePug">Robert Maddox</a> 2025
 * <p>
 * Methods that return strings for easily readable and fancy formatting in Telemetry
 */
// This originally created telemetry lines itself but I figured it'd make more sense for it to instead return a string
public class FancyFormatting {

    private static final double bar_width = 20; // The number of characters the progress bar is composed of // \u200B \u200C
    // Character unicodes
    private static final String unicode_green = "\uD83D\uDFE9";
    private static final String unicode_red = "\uD83D\uDFE5";
    private static final String unicode_a = "\uD83C\uDDE6";
    private static final String unicode_b = "\uD83C\uDDE7";
    private static final String unicode_c = "\uD83C\uDDE8";
    private static final String unicode_d = "\uD83C\uDDE9";
    private static final String unicode_e = "\uD83C\uDDEA";
    private static final String unicode_f = "\uD83C\uDDEB";
    private static final String unicode_g = "\uD83C\uDDEC";
    private static final String unicode_h = "\uD83C\uDDED";
    private static final String unicode_i = "\uD83C\uDDEE";
    private static final String unicode_j = "\uD83C\uDDEF";
    private static final String unicode_k = "\uD83C\uDDF0";
    private static final String unicode_l = "\uD83C\uDDF1";
    private static final String unicode_m = "\uD83C\uDDF2";
    private static final String unicode_n = "\uD83C\uDDF3";
    private static final String unicode_o = "\uD83C\uDDF4";
    private static final String unicode_p = "\uD83C\uDDF5";
    private static final String unicode_q = "\uD83C\uDDF6";
    private static final String unicode_r = "\uD83C\uDDF7";
    private static final String unicode_s = "\uD83C\uDDF8";
    private static final String unicode_t = "\uD83C\uDDF9";
    private static final String unicode_u = "\uD83C\uDDFA";
    private static final String unicode_v = "\uD83C\uDDFB";
    private static final String unicode_w = "\uD83C\uDDFC";
    private static final String unicode_x = "\uD83C\uDDFD";
    private static final String unicode_y = "\uD83C\uDDFE";
    private static final String unicode_z = "\uD83C\uDDFF";
    private static final String unicode_zwnj = "\u200C"; // used to prevent regional indicators from forming a flag (their entire purpose)


    /**
     *
     * @param value Decimal number between 0 and 1 to turn into a percentage value
     */
    public static String toStringPercent(double value) {
        double rounded_input = Math.min(Math.round(Math.abs(value)*bar_width), bar_width); // Turn the value from a decimal to an integer
        String result = "";

        for (int i = 0; i < rounded_input; i ++) { // Create the filled in bars
            result += "█";
        }
        for (int j = 0; j < bar_width-rounded_input; j++) { // Fill the rest with unfilled bars
            result += "░";
        }

        return(result);
    }
    public static String toStringBool(boolean value) {
        if (value) {
            return("██████████");
        } else {
            return("░░░░░░░░░░");
        }
    }
    public static String toStringBoolColored(boolean value) {
    if (value) {
        return(unicode_green+unicode_green+unicode_green+unicode_green+unicode_green+unicode_green+unicode_green+unicode_green+unicode_green+unicode_green+unicode_green+unicode_green+unicode_green+unicode_green+unicode_green);
    } else {
        return(unicode_red+unicode_red+unicode_red+unicode_red+unicode_red+unicode_red+unicode_red+unicode_red+unicode_red+unicode_red+unicode_red+unicode_red+unicode_red+unicode_red+unicode_red);
    }
}

    /**
     * Make a string more bold using Unicode regional indicator symbols
     * @return a bold-er string
     */
    public static String toBoldString(String input) {
        String out = "";
        for (int i = 0; i < input.length(); i++) {
            out += toBoldChar(input.charAt(i)) + unicode_zwnj;
        }
        return out;
    }

    /**
     * @param in Character to make bolder
     * @return a unicode for a regional indicator with the same character
     */
    public static String toBoldChar(char in) {
            in = Character.toLowerCase(in);
             switch (in) {
                 case 'a':
                    return unicode_a;
                 case 'b':
                     return unicode_b;
                 case 'c':
                     return unicode_c;
                 case 'd':
                     return unicode_d;
                 case 'e':
                     return unicode_e;
                 case 'f':
                     return unicode_f;
                 case 'g':
                     return unicode_g;
                 case 'h':
                     return unicode_h;
                 case 'i':
                     return unicode_i;
                 case 'j':
                     return unicode_j;
                 case 'k':
                     return unicode_k;
                 case 'l':
                     return unicode_l;
                 case 'm':
                     return unicode_m;
                 case 'n':
                     return unicode_n;
                 case 'o':
                     return unicode_o;
                 case 'p':
                     return unicode_p;
                 case 'q':
                     return unicode_q;
                 case 'r':
                     return unicode_r;
                 case 's':
                     return unicode_s;
                 case 't':
                     return unicode_t;
                 case 'u':
                     return unicode_u;
                 case 'v':
                     return unicode_v;
                 case 'w':
                     return unicode_w;
                 case 'x':
                     return unicode_x;
                 case 'y':
                     return unicode_y;
                 case 'z':
                     return unicode_z;
                 case ' ':
                     return "   ";
                 default:
                     return "？";
             }
        }



}
