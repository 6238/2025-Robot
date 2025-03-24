package frc.robot.util;

import static frc.robot.Constants.Swerve.MAX_SPEED;

import edu.wpi.first.math.filter.SlewRateLimiter;

public class DrivingRate {
    public static class DrivingRateConfig {
        public double CentreRate;
        public double MaxRate;
        public double Expo;

        public DrivingRateConfig(double CenterRate, double MaxRate, double Expo) {
            this.CentreRate = CenterRate;
            this.MaxRate = MaxRate;
            this.Expo = Expo;
        }
    }

    public static DrivingRateConfig scaleDrivingConfigs(double scale, DrivingRateConfig config) {
        config.MaxRate *= scale;
        config.Expo *= scale;
        config.CentreRate *= scale;
        return config;
    }

    public static double applyRateConfig(double x, DrivingRateConfig config) {
        double h = x * ( Math.pow(x, 5) * config.Expo ) + x * ( 1 - config.Expo );
        return ( config.CentreRate * x ) + ( ( config.MaxRate - config.CentreRate ) * h );
    }
}
