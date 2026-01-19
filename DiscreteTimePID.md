## Discrete PID controller approximation

Consider a continuous PID controller:

$$u(t) = K_p e(t) + \frac{K_p}{T_i}\int_0^t e(t) dt + K_p T_d \frac{d}{dt}e(t)$$

Where $K_p$ is the proportional gain, $T_i$ the integral time, and $T_d$ the derivative time.

In order to discretize the PID controller, let's assume a discrete time $t = kT$, where $T$ is the sample
period and $k$ the discrete-time index ($\mathbb{Z}_{\ge 0}$).

$$u(kT) = K_p e(kT) + \frac{K_p}{T_i}\int_{0}^{kT} e(t) dt + K_p T_d \frac{d}{dt}e(kT)$$

Then, using the ***trapezoidal approximation of the integral term and the finite difference approximation of the derivative
term***, the discrete form of the controller can be derived as:



https://github.com/user-attachments/assets/495cdff6-2721-4e73-b774-181abe9bd0c9



$$u[kT] = K_p e[kT] + \frac{K_p}{T_i}\frac{T}{2}\sum_{k=0}^{n}\left(e[(k-1)T] + e[kT]\right) + K_p T_d \frac{e[kT] - e[(k-1)T]}{T}$$

$$u[kT] = K_p e[kT] + \frac{K_p}{T_i}\frac{T}{2}\left(e[0] + 2e[T] + ... + 2e[(k-1)T] + e[kT]\right) + K_p T_d \frac{e[kT] - e[(k-1)T]}{T}$$

Since $T$ is a constant, the independent variable is simply $k$, so we can rewrite the previous equation as follows:

$$u[k] = K_p e[k] + \frac{K_p}{T_i}\frac{T}{2}\left(e[0] + 2e[1] + ... + 2e[k-1] + e[k]\right) + K_p T_d \frac{e[k] - e[k-1]}{T}$$

For $k-1$, we get:

$$u[k-1] = K_p e[k-1] + \frac{K_p}{T_i}\frac{T}{2}\left(e[0] + 2e[1] + ... + 2e[k-2] + e[k-1]\right) + K_p T_d \frac{e[k-1] - e[k-2]}{T}$$

Subtracting the second equation from the first, we obtain:

$$u[k] - u[k-1] =  K_p e[k] - K_p e[k-1]  + \frac{K_p}{T_i}\frac{T}{2} \left( e[k-1] + e[k] \right) +  K_p T_d \frac{\left(e[k] - 2e[k-1] + e[k-2]\right)}{T}$$

Finally, the discrete-time PID is given by:

$$\boxed{u[k] = u[k-1] + q_0 e[k] + q_1 e[k-1] + q_2 e[k-2]}$$

Where:

$$q_0 = K_p \left( 1 + \frac{T}{2 T_i} + \frac{T_d}{T}\right)$$

$$q_1 = -K_p \left( 1 - \frac{T}{2 T_i} + \frac{2T_d}{T} \right)$$

$$q_2 = \frac{K_p T_d}{T}$$

---

### Note 1:

Alternatively, an algebraic manipulation that leads to the same approximation is explained here.
First, let´s recall the *decomposition* property of the definite integral:

$$\frac{K_p}{T_i}\int_{0}^{kT} e(t) dt = \frac{K_p}{T_i} \left( \int_{0}^{(k-1)T} e(t) dt + \int_{(k-1)T}^{kT} e(t) dt \right)$$

Given that:

$$u([k-1]T) = K_p e([k-1]T) + \frac{K_p}{T_i}\int_{0}^{[k-1]T} e(t) dt + K_p T_d \frac{d}{dt}e([k-1]T)$$

Solving for the term $\frac{K_p}{T_i}\int_{0}^{[k-1]T} e(t) dt$ from the previous equation:

$$\frac{K_p}{T_i}\int_{0}^{[k-1]T} e(t) dt = u([k-1]T) - K_p e([k-1]T) - K_p T_d \frac{d}{dt}e([k-1]T)$$

The control output $u(kT)$ can be decomposed as follows:

$$u(kT) = K_p e(kT) + \frac{K_p}{T_i}\int_{0}^{(k-1)T} e(t) dt + \frac{K_p}{T_i}\int_{(k-1)T}^{kT} e(t) dt+ K_p T_d \frac{d}{dt}e(kT)$$

Since we already know what the term $\frac{K_p}{T_i}\int_{0}^{[k-1]T} e(t) dt$ is equivalent to, we can substitute it here:

$$u(kT) = K_p e(kT) + \left(u([k-1]T) - K_p e([k-1]T) - K_p T_d \frac{d}{dt}e([k-1]T)\right) + \frac{K_p}{T_i}\int_{(k-1)T}^{kT} e(t) dt+ K_p T_d \frac{d}{dt}e(kT)$$

Now, by applying the trapezoidal approximation to the integral term and the finite difference approximation to the derivative term, we finally obtain:

$$u[k] = K_p e[k] + u[k-1] - K_p e[k-1] - \frac{K_p T_d}{T} (e[k-1] - e[k-2]) + \frac{K_p}{T_i}\frac{T}{2} \left( e[k-1] + e[k] \right) + \frac{K_p T_d}{T} (e[k] - e[k-1])$$

$$\boxed{
u[k] = u[k-1] + K_p\left( 1 + \frac{T}{2T_i} + \frac{T_d}{T}\right)e[k] - K_p\left(1 - \frac{T}{2T_i} + \frac{2T_d}{T} \right)e[k-1] + \frac{K_p T_d}{T}e[k-2]
}$$

### Note 2:

If it is necessary to include an anti-windup mechanism, such as the clamping method, the controller is preferably expressed in its accumulator form:

$$\boxed{
u[k] = K_p e[k] + \frac{K_p}{T_i}\frac{T}{2}\sum_{k=0}^{n}\left(e[k-1] + e[k]\right) + K_p T_d \frac{e[k] - e[k-1]}{T}
}$$
