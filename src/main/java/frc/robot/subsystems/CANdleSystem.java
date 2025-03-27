package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CANdleSystem extends SubsystemBase{

    private static final CANdle candle = new CANdle(50);


    public static final int candleLed = 8;
    public static final int LedCountPerString = 30;
    public static final int ledStartStringOne = 8;
    public static final int ledStartStringTwo = 38;
    

    public CANdleSystem() {
        CANdleConfiguration config = new CANdleConfiguration();
        config.statusLedOffWhenActive = true;
        config.stripType = LEDStripType.GRB;
        config.v5Enabled = true;
        config.vBatOutputMode = CANdle.VBatOutputMode.Modulated;
        config.brightnessScalar = 1;
        candle.configAllSettings(config, 100);
        candle.configLEDType(LEDStripType.GRB); //just added this after cd post
        System.out.println("Candle Initialized");
    }

    public void SetLow(Color color)
    {
        LEDSegment.LowOne.setColor(color);
        LEDSegment.LowTwo.setColor(color);
        LEDSegment.LowThree.setColor(color);
        LEDSegment.LowFour.setColor(color);

    }
    public void SetMid(Color color)
    {
        LEDSegment.MidOne.setColor(color);
        LEDSegment.MidTwo.setColor(color);
        LEDSegment.MidThree.setColor(color);
        LEDSegment.MidFour.setColor(color);
    }
    public void SetHigh(Color color)
    {
        LEDSegment.HighOne.setColor(color);
        LEDSegment.HighTwo.setColor(color);
        LEDSegment.HighThree.setColor(color);
        LEDSegment.HighFour.setColor(color);
    }
    public void setRainbow() {
        LEDSegment.SegmentOne.setRainbowAnimation(1);
        LEDSegment.SegmentTwo.setRainbowAnimation(1);
        LEDSegment.SegmentThree.setRainbowAnimation(1);
        LEDSegment.SegmentFour.setRainbowAnimation(1);

    }
    public void setStrobeAnimation(Color color) {
        LEDSegment.SegmentOne.setStrobeAnimation(color, 1);
        LEDSegment.SegmentTwo.setStrobeAnimation(color,1);
        LEDSegment.SegmentThree.setStrobeAnimation(color,1);
        LEDSegment.SegmentFour.setStrobeAnimation(color,1);

    }

    public static enum LEDSegment {
        InternalLEDs(0, 8, 0),
        LowOne(8, 10, 1),
        MidOne(18, 10, 1),
        HighOne(28, 10, 1),
        HighTwo(38, 10, 1),
        MidTwo(48, 10, 1),
        LowTwo(58, 10, 1),
        LowThree(68, 10, 1),
        MidThree(78, 10, 1),
        HighThree(88, 10, 1),
        HighFour(98, 10, 1),
        MidFour(108, 10, 1),
        LowFour(118, 10, 1),
        SegmentOne(8,30,2),
        SegmentTwo(3,30,3),
        SegmentThree(68,30,4),
        SegmentFour(98,30,5);

        //start index is what LED to start on, 0-7 are the candles onboard LEDS, beyond is the strip

        public final int startIndex;
        public final int segmentSize;
        public final int animationSlot;

        private LEDSegment(int startIndex, int segmentSize, int animationSlot) {
            this.startIndex = startIndex;
            this.segmentSize = segmentSize;
            this.animationSlot = animationSlot;
        }

        public void setColor(Color color) {
            clearAnimation();
            candle.setLEDs(color.red, color.green, color.blue, 0, startIndex, segmentSize);
        }

        private void setAnimation(Animation animation) {
            candle.animate(animation, animationSlot);
        }

        public void fullClear() {
            clearAnimation();
            disableLEDs();
        }

        public void clearAnimation() {
            candle.clearAnimation(animationSlot);
        }

        public void disableLEDs() {
            setColor(Constants.Candle.black);
        }

        public void setFlowAnimation(Color color, double speed) {
            setAnimation(new ColorFlowAnimation(
                    color.red, color.green, color.blue, 0, speed, segmentSize, Direction.Forward, startIndex));
        }

        public void setFadeAnimation(Color color, double speed) {
            setAnimation(
                    new SingleFadeAnimation(color.red, color.green, color.blue, 0, speed, segmentSize, startIndex));
        }

        public void setBandAnimation(Color color, double speed) {
            setAnimation(new LarsonAnimation(
                    color.red, color.green, color.blue, 0, speed, segmentSize, BounceMode.Front, 3, startIndex));
        }

        public void setStrobeAnimation(Color color, double speed) {
            setAnimation(new StrobeAnimation(color.red, color.green, color.blue, 0, speed, segmentSize, startIndex));
        }

        public void setRainbowAnimation(double speed) {
            setAnimation(new RainbowAnimation(1, speed, segmentSize, false, startIndex));
        }

        public void setFireAnimation(double speed, double cooling, double sparking) {
            setAnimation(new FireAnimation(1, speed, segmentSize, cooling, sparking));

        }

        public void setDefaultLEDColors() {
        }
    }

    public static class Color {
        public int red;
        public int green;
        public int blue;

        public Color(int red, int green, int blue) {
            this.red = red;
            this.green = green;
            this.blue = blue;
        }
        
    }
}