# used to specify what is available to import from this file
__all__ = ["DiscretePID"]


class DiscretePID:
    """
    Modified discrete PID controller using a parallel implementation with a
    bilinear integrator and a backward euler filtered derivative.
    
    For a more detailed explanation please see the following documentation:
    https://autoboat-vt.github.io/documentation/ros2_packages/autopilot_package/pidn_controller_derivation_and_explanation

    I spent a lot of time on that documentation so it would be nice if you would take a look at it :)
    
    Parameters
    ----------
    sample_period
        Sample period for the discrete PID controller.
    k_p
        Proportional gain.
    k_i
        Integral gain.
    k_d
        Derivative gain.
    n
        Cutoff frequency for the low pass filter on the derivative term. Lower value means more smoothing.
    b_i
        Maximum saturation bound for the integral state to prevent stall windup.
    """

    def __init__(
        self,
        sample_period: float,
        k_p: float,
        k_i: float,
        k_d: float,
        n: float,
        b_i: float = 15, # could also default to float("inf") if you want
    ) -> None:
        self.sample_period = sample_period
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d
        self.n = n
        self.b_i = b_i

        self.prev_integral: float = 0.0
        self.prev_derivative: float = 0.0
        self.prev_error: float = 0.0

    def reset(self) -> None:
        """Resets the PID controller to its initial state."""

        self.__init__(
            self.sample_period,
            self.k_p,
            self.k_i,
            self.k_d,
            self.n,
            self.b_i,
        )

    def __call__(self, error: float) -> float:
        """
        Calls :meth:`step` with the given error and returns the output.

        Parameters
        ----------
        error
            The current error value to be processed by the PID controller.

        Returns
        -------
        `float`
            The output of the PID controller after processing the input error.
        """

        return self.step(error)


    def set_gains(
        self,
        k_p: float | None = None,
        k_i: float | None = None,
        k_d: float | None = None,
        n: float | None = None,
        b_i: float | None = None,
        sample_period: float | None = None,
    ) -> None:
        """
        Sets gains to the specified value.

        Parameters
        ----------
        k_p
            Proportional gain.
        k_i
            Integral gain.
        k_d
            Derivative gain.
        n
            Cutoff frequency for the low pass filter on the derivative term.
        b_i
            Maximum saturation bound for the integral state.
        sample_period
            Sample period for the discrete PID controller.
        """

        if k_p is not None:
            self.k_p = k_p
        if k_i is not None:
            self.k_i = k_i
        if k_d is not None:
            self.k_d = k_d
        if n is not None:
            self.n = n
        if b_i is not None:
            self.b_i = b_i
        if sample_period is not None:
            self.sample_period = sample_period


    def step(self, error: float) -> float:
        """
        Steps the PID controller with the given error and returns the output.

        Parameters
        ----------
        error
            The current error value to be processed by the PID controller.

        Returns
        -------
        `float`
            The output of the PID controller after processing the input error.
        """

        # Proportional Term
        p_output = self.k_p * error

        # Integral term with clamping
        integral = self.prev_integral + (self.k_i * self.sample_period / 2.0) * (error + self.prev_error)
        i_output = max(-self.b_i, min(self.b_i, integral))

        # Derivative term
        d_denom = 1.0 + self.n * self.sample_period
        d_output = (self.prev_derivative + self.k_d * self.n * (error - self.prev_error)) / d_denom

        output = p_output + i_output + d_output

        self.prev_integral = i_output
        self.prev_derivative = d_output
        self.prev_error = error

        return output
