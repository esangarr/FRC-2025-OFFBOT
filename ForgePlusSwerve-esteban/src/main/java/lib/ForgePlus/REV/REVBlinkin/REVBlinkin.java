package lib.ForgePlus.REV.REVBlinkin;

import java.util.Optional;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import lib.ForgePlus.Equals.Epsilon;

public class REVBlinkin implements Sendable{

    private final Spark signal;

    public enum PatternType{

        RainbowRainbowPalette(-0.99), 
        RainbowPartyPalette(-.97), 
        RainbowOceanPalette(-.95), 
        RainbowLavaPalette(-.93), 
        RainbowForestPalette(-.91), 
        RainbowGlitter(-.89), 
        Confetti(-.87),
        ShotRed(-.85),
        ShotBlue(-.83),
        ShotWhite(-.81),
        SinelonRainbowPalette(-.79),
        SinelonPartyPalette(-.77),
        SinelonOceanPalette(-.75),
        SinelonLavaPalette(-.73),
        SinelonForestPalette(-.71),
        BPMRainbowPalette(-.69),
        BPMPartyPalette(-.67),
        BPMOceanPalette(-.65),
        BPMLavaPalette(-.63),
        BPMForestPalette(-.61),
        FireMedium(-.59),
        FireLarge(-.57),
        TwinklesRainbowPalette(-.55),
        TwinklesPartyPalette(-.53),
        TwinklesOceanPalette(-.51),
        TwinklesLavaPalette(-.49),
        TwinklesForestPalette(-.47),
        ColorWavesRainbowPalette(-.45),
        ColorWavesPartyPalette(-.43),
        ColorWavesOceanPalette(-.41),
        ColorWavesLavaPalette(-.39),
        ColorWavesForestPalette(-.37),
        LarsonScannerRed(-.35),
        LarsonScannerGray(-.33),
        LightChaseRed(-.31),
        LightChaseBlue(-.29),
        LightChaseGray(-.27),
        HeartbeatRed(-.25),
        HeartbeatBlue(-.23),
        HeartbeatWhite(-.21),
        HeartbeatGray(-.19),
        BreathRed(-.17),
        BreathBlue(-.15),
        BreathGray(-.13),
        StrobeRed(-.11),
        StrobeBlue(-.09),
        StrobeGold(-.07),
        StrobeWhite(-.05),
        C1EndToEndBlendToBlack(-.03),
        C1LarsonScanner(-.01),
        C1LightChase(.01),
        C1HeartbeatSlow(.03),
        C1HeartbeatMedium(.05),
        C1HeartbeatFast(.07),
        C1BreathSlow(.09),
        C1BreathFast(.11),
        C1Shot(.13),
        C1Strobe(.15),
        C2EndToEndBlendToBlack(.17),
        C2LarsonScanner(.19),
        C2LightChase(.21),
        C2HeartbeatSlow(.23),
        C2HeartbeatMedium(.25),
        C2HeartbeatFast(.27),
        C2BreathSlow(.29),
        C2BreathFast(.31),
        C2Shot(.33),
        C2Strobe(.35),
        SparkleC1onC2(.37),
        SparkleC2onC1(.39),
        ColorGradientC1toC2(.41),
        BPMC1toC2(.43),
        EndToEndBlendC1toC2(.45),
        EndToEndBlend(.47),
        C1toC2NoBlending(.49),
        TwinkleC1toC2(.51),
        ColorWavesC1toC2(.53),
        SinelonC1toC2(.55),
        HotPink(.57),
        DarkRed(.59),
        Red(.61),
        RedOrange(.63),
        Orange(.65),
        Gold(.67),
        Yellow(.69),
        LawnGreen(.71),
        Lime(.73),
        DarkGreen(.75),
        Green(.77),
        BlueGreen(.79),
        Aqua(.81),
        SkyBlue(.83),
        DarkBlue(.85),
        Blue(0.87),
        BlueViolet(.89),
        Violet(.91),
        White(.93),
        Gray(.95),
        DarkGray(.97),
        Black(.99),
        None(0);

        public final double Value;

        private PatternType(double Value){
            this.Value = Value;
        }

        public static PatternType fromValue(double value) {
            for (PatternType pattern : PatternType.values()) {
                if (Epsilon.equals(value, pattern.Value)) {
                    return pattern;
                }
            }
            return None;
        }

        public static PatternType fromName(String name) {
            for (PatternType pattern : PatternType.values()) {
                if (name.equals(pattern.name())) {
                    return pattern;
                }
            }
            return None;
        }

    }

    private boolean off = false;
    private PatternType lastPattern = PatternType.None;

    public REVBlinkin(int PWMport){
        this.signal = new Spark(PWMport);
        this.off = false;
        setPattern(lastPattern);
        
    }

    public void disable(boolean value){
        this.off = value;
        if (value) {
            signal.set(PatternType.Black.Value);
        }
    }

    public void on(){
        disable(false);
    }

    public void off(){
        disable(true);
    }

    public void toggle() {
        if (off) {
            on();
        } else {
            off();
        }
    }

    public boolean isOn(){
        return !off;
    }

    public boolean isConnected(){
        return signal.isAlive();
    }

    private void handlePattern(PatternType pattern) {

        if (off){
            return;
        }

        if (!lastPattern.equals(pattern)) {

            this.lastPattern = pattern;        
            signal.set(pattern.Value);
        }
    }

    public void setPattern(PatternType pattern){
        handlePattern(pattern);
    }

    public void setPattern(double value){
        handlePattern(PatternType.fromValue(value));
        
    }

    public void setPattern(String name){
        handlePattern(PatternType.fromName(name));
    }

    public Optional<PatternType> getPattern(){
        
        for (PatternType pattern : PatternType.values()) {
            if (Epsilon.equals(signal.get(), pattern.Value)) {
                return Optional.of(pattern);
            }
        }
        return Optional.empty();
    }

    public PatternType getLastPattern(){
        return lastPattern;
    }

    public boolean hasPattern(PatternType pattern){
        return Epsilon.equals(signal.get(), pattern.Value);
    }

    @Override
    public void initSendable(SendableBuilder builder){

        builder.setSmartDashboardType("REVBlinkin");
        builder.addStringProperty("Pattern", ()-> getLastPattern().toString(), this::setPattern);
        builder.addDoubleProperty("Signal Value", signal::get, this::setPattern);
        builder.addBooleanProperty("Connected", this::isConnected, null);
        builder.addBooleanProperty("Enabled", this::isOn, this::disable);
        
    }

}