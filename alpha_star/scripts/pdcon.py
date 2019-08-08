from __future__ import division
import time


class pd_controller:

    def __init__(self, setpoint = 1, kp = 1, kd = 1, derivative_samples = 3):
        self.setpoint = setpoint
        self._der = 0
        self.kp = kp
        self.kd = kd
        self.derivative_samples = derivative_samples
        self._curtime = time.time()
        self._history = [0 for i in range(self.derivative_samples)]
        self._history_times = [0 for i in range(self.derivative_samples)]


    def _calc_der(self):
        self._der = 0
        c = 0
        for i in range(len(self._history)-1):
            for j in range(i + 1, len(self._history)):
                if self._history_times[j] > self._history_times[i]:
                    self._der += (self._history[j] - self._history[i]) / (self._history_times[j] - self._history_times[i])
                    c+=1
        self._der /= float(c)


    def pd_out(self, out):
        self._history.pop(0)
        self._history.append(out - self.setpoint)
        self._history_times.pop(0)
        self._history_times.append(time.time())
        self._calc_der()
        return self.kp*self._history[-1] + self.kd*self._der, self.kp*self._history[-1], self.kd*self._der

    def reset(self, sp):
        self.setpoint = sp
        self._der = 0
        self._curtime = time.time()
        self._history = [0 for i in range(self.derivative_samples)]
        self._history_times = [0 for i in range(self.derivative_samples)]


if __name__ == "__main__":
    from matplotlib import pyplot as plt
    mypd = pd_controller(kp = 0.01, kd = 5)
    t = [i for i in range(3000)]
    x = [0]
    dt = 0.001
    err = 0.1
    vel = 0
    for i in t[1:]:
        out = mypd.pd_out(x[-1])
        vel += out
        x.append(x[-1] + vel*dt)
        time.sleep(dt)
    plt.plot(t,x)
    plt.show()
