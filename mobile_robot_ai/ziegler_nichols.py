import numpy as np

# Global variables for K_u and P_u


class ZieglerNichols():
    def __init__(self):
        self.Ku = 0.0
        self.Pu = None
        self.prev_outputs = []
        self.prev_peak = None
        self.prev_trough = None
        self.prev_peak_time = None
        self.prev_trough_time = None
        self.fail_count = 0

    def ziegler_nichols_tuning(self, control_input, time, Ku_increase=0.05, tolerance=0.05, window_size=10, failure_threshold=10):
        # If P_u is already calculated, return 
        if self.Pu != None:
            return
        
        # Update the previous outputs history list
        self.prev_outputs.append(control_input)

        # Check if enough data points are provided
        if len(self.prev_outputs) < window_size:
            print("Not enough data provided")
            return # Not enough data to evaluate

        # Calculate the standard deviation of the outputs
        std_dev = np.std(self.prev_outputs)

        # Check if the standard deviation is within tolerance
        is_steady = std_dev < tolerance

        if is_steady:
            # Update K_u only if the systeis_steady is steady
            self.Ku += Ku_increase # Increment Ku by the tolerance value

            # Detect peak and trough
            if self.prev_peak is None or control_input > self.prev_outputs[-2]:
                self.prev_peak = control_input
                self.prev_peak_time = time
            elif self.prev_trough is None or control_input < self.prev_outputs[-2]:
                self.prev_trough = control_input
                self.prev_trough_time = time

            #previous_outputs[i] = current_output  # Update previous output
        else:
            # Increment failure count if not steady
            self.fail_count += 1

            if self.fail_count >= failure_threshold:
                # Calculate P_u only if both peak and trough have been calculated
                if self.prev_peak is not None and self.prev_trough is not None:
                    self.Pu = abs(self.prev_peak_time - self.prev_trough_time)
                    print("K_u: ", self.Ku)
                    print("P_u: ", self.Pu)
                else:
                    print("P_u cannot be calculated.\n"+
                          "Try to modify the the Ku_increase, tolerance, window_size and/or failure_threshold parameters.")
        
        return 
