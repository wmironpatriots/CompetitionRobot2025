package lib.utils;

import edu.wpi.first.math.MathUtil;

public class ControllerUtil {
    /**
     * Squares and deadbands a joystick value
     *
     * @param val joystick value
     * @return modified joystick value
     */
    public static double modifyJoystick(double val) {
        return MathUtil.applyDeadband(Math.abs(Math.pow(val, 2)) * Math.signum(val), 0.08);
    } 
}
