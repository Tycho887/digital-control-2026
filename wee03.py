import numpy as np

# Given a linear difference equation of the form: 
# u_k = a_1 * u_{k-1} + a_2 * u_{k-2} + ... + a_n * u_{k-n} + b
# The characteristic polynomial is given by:
# P(x) = x^n - a_1 * x^(n-1)
# Our goal is to find the roots of the polynomial, and check that they are all within
# the unit circle (i.e., their absolute values are less than 1).

class CCDE:
    def __init__(self, coefficients: np.ndarray):
        """
        We assume the coefficients represent the linear case:
        a_1 u_k + a_2 u_{k-1} + ... + a_n u_{k-n} = 0
        """
        self.coefficients = coefficients
        self.roots = None

    def find_roots(self):
        # Find the roots of the characteristic polynomial
        poly = np.poly1d(self.coefficients)
        self.roots = poly.r
        return self.roots

    def check_stability(self):
        # Check if all roots are within the unit circle
        if self.roots is None:
            self.find_roots()
        return all(abs(root) < 1 for root in self.roots)

def part_1_tests():
    case_1a = np.array([1, -0.5, 0.3])
    case_1b = np.array([1, -1.6, 1])
    case_1c = np.array([1, -0.8, 0.4])
    for case in [case_1a, case_1b, case_1c]:
        ccde = CCDE(case)
        roots = ccde.find_roots()
        print(f"Roots of {case}: {roots}")
        print(f"Is {case} stable? {ccde.check_stability()}")

def part_2_tests():
    # Here we should find the characteristic equations in Z for the following difference equations:
    # 1. u_k = 0.25 u_{k-1}
    # 2. u_k = -0.25 u_{k-1}
    # 3. u_k = u_{k-1} - 0.5 u_{k-2}

    # Using the z-transform: 
    # U(z) = sum_{k=0}^{\infty} u_k z^{-k}
    # For 2A this gives: U(z) = 0.25 z^{-1} U(z) => (1 - 0.25 z^{-1}) U(z) = 0 => P(z) = 1 - 0.25 z^{-1}
    # For 2B this gives: U(z) = -0.25 z^{-1} U(z) => (1 + 0.25 z^{-1}) U(z)
    # For 2C this gives: U(z) = z^{-1} U(z) - 0.5 z^{-2} U(z) => (1 - z^{-1} + 0.5 z^{-2}) U(z) = 0 => P(z) = 1 - z^{-1} + 0.5 z^{-2}

    # To determine stability, we can check the roots of the Polynomial P(z) for each case. If all roots are inside the unit circle, the system is stable.
    case_2a = np.array([1, -0.25])
    case_2b = np.array([1, 0.25])
    case_2c = np.array([1, -1, 0.5])
    for case in [case_2a, case_2b, case_2c]:
        ccde = CCDE(case)
        roots = ccde.find_roots()
        print(f"Roots of {case}: {roots}")
        print(f"Is {case} stable? {ccde.check_stability()}")
    

def part_3_tests():
    case_3a = np.array([1, -1.1, 0.01, 0.405])
    case_3b = np.array([1, -3.6, 4, -1.6])
    for case in [case_3a, case_3b]:
        ccde = CCDE(case)
        roots = ccde.find_roots()
        print(f"Absolute values of roots for {case}: {np.abs(roots)}")
        # Number of roots outside the unit circle
        num_outside = sum(abs(root) >= 1 for root in roots)
        print(f"Number of roots outside the unit circle: {num_outside}")

if __name__ == "__main__":
    print("Running Part 1 Tests:")
    part_1_tests()
    print("Running Part 2 Tests:")
    part_2_tests()
    print("Running Part 3 Tests:")
    part_3_tests()