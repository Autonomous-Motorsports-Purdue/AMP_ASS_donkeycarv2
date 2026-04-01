from datetime import datetime
class Logger2:
    def __init__(self):
        self.path = "log.txt"
    
    def run(self, **kwargs):
        for name, arg in kwargs.items():
            with open(self.path, "a") as file:
                file.write(f"[{datetime.now()}] [{name}]: {arg}\n")
