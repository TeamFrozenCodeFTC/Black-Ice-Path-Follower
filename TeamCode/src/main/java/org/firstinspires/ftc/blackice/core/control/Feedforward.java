package org.firstinspires.ftc.blackice.core.control;

import org.firstinspires.ftc.blackice.util.Tunable;

@FunctionalInterface
public interface Feedforward {
    double compute(double target);
    
    static Feedforward zero() {
        return target -> 0;
    }
    
    class Linear implements Feedforward {
        @Tunable
        double kV, kS;
        
        public Linear(double kV, double kS) {
            this.kV = kV;
            this.kS = kS;
        }
        
        @Override
        public double compute(double target) {
            return kV * target + kS * Math.signum(target);
        }
    }
}
