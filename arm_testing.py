from turtle import * 
# from vpython import *

joint1 = float(input("joint 1 angle: "))
joint2 = float(input("joint 2 angle: "))
left(90)
forward(100)
right(180)
left(joint1)
forward(100)
right(180-joint2)
forward(100)
done()

# box(color = vector(1, 0.6, 0), 
#     opacity = 0.5, 
#     shininess = 1, 
#     emissive = False)