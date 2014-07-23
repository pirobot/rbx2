#!/usr/bin/env python

import time
import ecto
from ecto.opts import scheduler_options, run_plasm
from ecto_opencv.highgui import VideoCapture, imshow, FPSDrawer
from ecto_opencv.imgproc import Canny
import argparse

parser = argparse.ArgumentParser(description='My awesome program thing.')

scheduler_options(parser)
options = parser.parse_args()

video_cap = VideoCapture(video_device=0, width=320, height=240)
fps = FPSDrawer()
edges = Canny(threshold1=100.0, threshold2 = 50.0)

plasm = ecto.Plasm()
plasm.connect(video_cap['image'] >> fps['image'],
              fps['image'] >> edges[:],
              edges[:] >> imshow(name='video_cap')['image'],
              )

#run_plasm(options, plasm, locals=vars())

sched = ecto.Scheduler(plasm)
sched.execute_async()
                       
time.sleep(5)

#sched.stop()

#sched.wait()
#threshold1 = 1.0
#sched.execute_async()
#time.sleep(5)
#sched.stop()
#threshold1 = 100.0
#sched.execute_async()



#sched.execute()