package pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;

public class LConstants {
    static {
        ThreeWheelConstants.forwardTicksToInches = 0.00056050;
        ThreeWheelConstants.strafeTicksToInches = -0.00056208;
        ThreeWheelConstants.turnTicksToInches = 0.0005589587;
        ThreeWheelConstants.leftY = 6.109;
        ThreeWheelConstants.rightY = -6.109;
        ThreeWheelConstants.strafeX = -6.391;
        ThreeWheelConstants.leftEncoder_HardwareMapName = "fl";
        ThreeWheelConstants.rightEncoder_HardwareMapName = "fr";
        ThreeWheelConstants.strafeEncoder_HardwareMapName = "br";
        ThreeWheelConstants.leftEncoderDirection = Encoder.REVERSE;
        ThreeWheelConstants.rightEncoderDirection = Encoder.FORWARD;
        ThreeWheelConstants.strafeEncoderDirection = Encoder.FORWARD;
    }
}




