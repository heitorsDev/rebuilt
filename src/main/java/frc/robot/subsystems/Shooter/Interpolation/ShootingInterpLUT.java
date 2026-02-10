package frc.robot.subsystems.Shooter.Interpolation;

import java.util.Arrays;
import java.util.Comparator;

public final class ShootingInterpLUT {
    //VIBE CODED
    private final double[] inputs;
    private final double[] outputs;
    private final int size;

    public ShootingInterpLUT(double[][] registers) {
        if (registers == null || registers.length < 2) {
            throw new IllegalArgumentException("Interpolation requires at least 2 points.");
        }
        double[][] sorted = Arrays.copyOf(registers, registers.length);
        Arrays.sort(sorted, Comparator.comparingDouble(a -> a[0]));

        size = sorted.length;
        inputs = new double[size];
        outputs = new double[size];

        for (int i = 0; i < size; i++) {
            inputs[i] = sorted[i][0];
            outputs[i] = sorted[i][1];
        }
    }

    public double get(double in) {
        if (in <= inputs[0]) return outputs[0];
        if (in >= inputs[size - 1]) return outputs[size - 1];
        int low = 0;
        int high = size - 1;

        while (low <= high) {
            int mid = (low + high) >>> 1;

            if (inputs[mid] < in) {
                low = mid + 1;
            } else if (inputs[mid] > in) {
                high = mid - 1;
            } else {
                return outputs[mid];
            }
        }
        
        int lowerIndex = high;
        int upperIndex = low;

        double x1 = inputs[lowerIndex];
        double y1 = outputs[lowerIndex];
        double x2 = inputs[upperIndex];
        double y2 = outputs[upperIndex];
        
        return y1 + (in - x1) * (y2 - y1) / (x2 - x1);
    }

}
