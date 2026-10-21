## Exercise 1: Stability Check

* **a) Equation:** $u_{k}=0.5u_{k-1}-0.3u_{k-2}$

* **Characteristic Polynomial:** $z^2 - 0.5z + 0.3 = 0$
* **Program Output:** Roots are $0.25 \pm 0.4873j$.
* **Conclusion:** Stable (True).


* **b) Equation:** $u_{k}=1.6u_{k-1}-u_{k-2}$

* **Characteristic Polynomial:** $z^2 - 1.6z + 1 = 0$
* **Program Output:** Roots are $0.8 \pm 0.6j$. The magnitude is $\sqrt{0.8^2 + 0.6^2} = 1$.
* **Conclusion:** Unstable (False), as roots are on the unit circle.


* **c) Equation:** $u_{k}=0.8u_{k-1}+0.4u_{k-2}$.


* **Characteristic Polynomial:** $z^2 - 0.8z - 0.4 = 0$.
* **Program Output:** Roots are 1.14833148 and -0.34833148.
* **Conclusion:** Unstable (False), as the magnitude of 1.14833148 is greater than 1, placing the root outside the unit circle.
## Exercise 2: Difference Equations Analysis

**a) Equation:** $u_{k+2}=0.25u_{k}$

* **1) Characteristic Equation:** $z^2 - 0.25 = 0$
* **2) Stability:** The analytical roots are $z_1 = 0.5$ and $z_2 = -0.5$. Since both magnitudes are strictly less than 1, the system is **Stable**.
* **3) Matching Coefficients:**
Using the general solution $u_k = A_1(0.5)^k + A_2(-0.5)^k$ and initial conditions:
$u_0 = 0 \implies A_1 + A_2 = 0$
$u_1 = 1 \implies 0.5A_1 - 0.5A_2 = 1$
Solving the system yields $A_1 = 1$ and $A_2 = -1$.

**b) Equation:** $u_{k+2}=-0.25u_{k}$

* **1) Characteristic Equation:** $z^2 + 0.25 = 0$
* **2) Stability:** The analytical roots are $z_1 = 0.5j$ and $z_2 = -0.5j$. The system is **Stable**.
* **3) Matching Coefficients:**
Using the general solution $u_k = A_1(0.5j)^k + A_2(-0.5j)^k$ and initial conditions:
$u_0 = 0 \implies A_1 + A_2 = 0$
$u_1 = 1 \implies A_1(0.5j) + A_2(-0.5j) = 1$
Substituting $A_2 = -A_1$ gives $A_1(0.5j) - A_1(-0.5j) = 1$, which simplifies to $A_1(j) = 1$. The final constants are $A_1 = -j$ and $A_2 = j$.

**c) Equation:** $u_{k+2}=u_{k+1}-0.5u_{k}$

* **1) Characteristic Equation:** $z^2 - z + 0.5 = 0$
* **2) Stability:** The program evaluated `[1. -1. 0.5]` correctly. The roots are $0.5 \pm 0.5j$. Since the magnitude of these roots is $\sqrt{0.5^2 + 0.5^2} \approx 0.707 < 1$, the system is **Stable**.
* **3) Matching Coefficients:**
Using the general solution $u_k = A_1(0.5 + 0.5j)^k + A_2(0.5 - 0.5j)^k$ and initial conditions:
$u_0 = 0 \implies A_1 + A_2 = 0 \implies A_2 = -A_1$
$u_1 = 1 \implies A_1(0.5 + 0.5j) - A_1(0.5 - 0.5j) = 1$
This simplifies to $A_1(j) = 1$. The final constants are $A_1 = -j$ and $A_2 = j$.

---

## Exercise 3: Roots Outside the Unit Circle

* **a) Equation:** $z^3-1.1z^2+0.01z+0.405=0$

* **Absolute values of roots:** $0.9, 0.9, 0.5$
* **Result:** **0** roots are outside the unit circle.


* **b) Equation:** $z^3-3.6z^2+4z-1.6=0$

* **Absoloute values of roots:** $2.0, 0.8944, 0.8944$
* **Result:** **1** root is outside the unit circle.