from subprocess import Popen, PIPE
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from threading import Thread
from collections import deque

xs = deque(maxlen=2000)
yset = deque(maxlen=2000)
ymeas = deque(maxlen=2000)
done = False

fig = plt.figure()
ax1 = fig.add_subplot(1, 1, 1)


def animate(i, xs, ymeas, yset):
    ax1.clear()
    ax1.plot(xs, ymeas, label="measured")
    ax1.plot(xs, yset, "-.", label="setpoint")


def read():
    global xs
    global ymeas
    global yset
    global done

    with Popen(["sudo", "./test"], stdout=PIPE) as p:
        try:
            print("starting live plot...")
            i = 0
            while True and not done:
                text = p.stdout.readline().decode("utf-8")

                # PID: meas 0.000000, out 0.000000, dir 1, duty 0
                if "PID" in text:
                    text = text.split()
                    measured = float(text[2][:-1])
                    setpoint = float(text[10])
                    i += 1

                    ymeas.append(measured)
                    yset.append(setpoint)
                    xs.append(i)
        except KeyboardInterrupt:
            pass


thread = Thread(target=read)
thread.start()

ani = animation.FuncAnimation(
    fig=fig, func=animate, interval=0.5, fargs=(xs, ymeas, yset)
)
plt.show()

done = True

thread.join()
