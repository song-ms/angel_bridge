from time import sleep
from math import pi, sin
from api_demo.sine_api import *


def main():
    app = API('test_app')

    app.SetFreq(1)

    for _ in range(20):
        pos = app.ReadWave()
        print(pos)
        sleep(0.01)

    app.Terminate()



if __name__ == "__main__":
    main()