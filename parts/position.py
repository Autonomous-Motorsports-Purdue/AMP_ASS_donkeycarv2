import time

class Position:
    def __init__(self):
        self.vx = 0.0
        self.vy = 0.0
        self.px = 0.0
        self.py = 0.0

    def run(self, ax, ay):
        time_step = .01
        self.vx += ax * time_step
        self.vy += ay * time_step
        self.px += self.vx * time_step
        self.py += self.vy * time_step
        print(f"px = {self.px:.4f}, py = {self.py:.4f}, ax = {ax:.4f}, ay = {ay:.4f}")
        return self.px, self.py


# ---------- CONFIG ----------
#dt = 0.01  # timestep (seconds)
# ---------------------------

#pos = Position(ax=1.0, ay=0.5)

#last_update = time.time()

#while True:
    #now = time.time()
   # if now - last_update >= dt:
        #px, py = pos.run(dt)

       # print(f"px = {px:.4f}, py = {py:.4f}, ax = {ax:.4f}, ay = {ay:.4f}")

       # last_update = now


