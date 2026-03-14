package org.firstinspires.ftc.teamcode.v2.lib;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

public class CommandGamepad extends GamepadEx {
    /**
     * The constructor, that contains the gamepad object from the
     * opmode.
     *
     * @param gamepad the gamepad object from the opmode
     */
    public CommandGamepad(Gamepad gamepad) {
        super(gamepad);
    }

    public GamepadButton y() { return getGamepadButton(GamepadKeys.Button.Y); }
    public GamepadButton x() { return getGamepadButton(GamepadKeys.Button.X); }
    public GamepadButton a() { return getGamepadButton(GamepadKeys.Button.A); }
    public GamepadButton b() { return getGamepadButton(GamepadKeys.Button.B); }

    public GamepadButton leftBumper() { return getGamepadButton(GamepadKeys.Button.LEFT_BUMPER); }
    public GamepadButton rightBumper() { return getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER); }

    public GamepadButton start() { return getGamepadButton(GamepadKeys.Button.START); }
    public GamepadButton back() { return getGamepadButton(GamepadKeys.Button.BACK); }
    public GamepadButton options() { return getGamepadButton(GamepadKeys.Button.OPTIONS); }

    public GamepadButton dpadUp() { return getGamepadButton(GamepadKeys.Button.DPAD_UP); }
    public GamepadButton dpadDown() { return getGamepadButton(GamepadKeys.Button.DPAD_DOWN); }
    public GamepadButton dpadLeft() { return getGamepadButton(GamepadKeys.Button.DPAD_LEFT); }
    public GamepadButton dpadRight() { return getGamepadButton(GamepadKeys.Button.DPAD_RIGHT); }

    public GamepadButton leftStick() { return getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON); }
    public GamepadButton rightStick() { return getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON); }


    public GamepadButton triangle() { return getGamepadButton(GamepadKeys.Button.TRIANGLE); }
    public GamepadButton square() { return getGamepadButton(GamepadKeys.Button.SQUARE); }
    public GamepadButton cross() { return getGamepadButton(GamepadKeys.Button.CROSS); }
    public GamepadButton circle() { return getGamepadButton(GamepadKeys.Button.CIRCLE); }
    public GamepadButton ps() { return getGamepadButton(GamepadKeys.Button.PS); }
    public GamepadButton SHARE() { return getGamepadButton(GamepadKeys.Button.SHARE); }
    public GamepadButton touchpad() { return getGamepadButton(GamepadKeys.Button.TOUCHPAD);}
    public GamepadButton touchpadFinger1() { return getGamepadButton(GamepadKeys.Button.TOUCHPAD_FINGER_1);}
    public GamepadButton touchpadFinger2() { return getGamepadButton(GamepadKeys.Button.TOUCHPAD_FINGER_2);}

}
