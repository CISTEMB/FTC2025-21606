package org.firstinspires.ftc.teamcode;

import com.bylazar.lights.RGBIndicator;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class GoBildaRGBIndicator {
    public enum Color {
        Off(0.0),
        Red(0.3),
        Orange(0.333),
        Yellow(0.388),
        Sage(0.444),
        Green(0.5),
        Azure(0.555),
        Blue(0.611),
        Indigo(0.666),
        Violet(0.722),
        White(1.0);
        public final double position;

        Color(double position) {
            this.position = position;
        }
    }

    //
    // Hardware
    //
    private final Servo servo;

    //
    // Panels
    //
    private final RGBIndicator panelsRGB;

    public RGBIndicator getPanels() {
        return panelsRGB;
    }

    public GoBildaRGBIndicator(HardwareMap hw, String name) {
        this.servo = hw.get(Servo.class, name);

        panelsRGB = new RGBIndicator(name);
    }

    public void set(Color color) {
        if (color == null) {
            color = Color.Off;
        }

        servo.setPosition(color.position);
        panelsRGB.update(color.position);
    }
}