from Tkinter import *
import simulator

def makeSim(world):
    canvasWidth = world.width
    canvasHeight = world.height
    master = Tk()
    canvas = Canvas(master, width=canvasWidth, height=canvasHeight)
    canvas.pack()
    sim = simulator.Simulator(canvas, canvasWidth, canvasHeight)
    return sim

def drawRobotStartGoalObstacles(sim, robot, obstacles, start, goal):
    sim.drawObstacles(obstacles)
    robot.moveToConfiguration(start)
    sim.drawRobot(robot)
    robot.moveToConfiguration(goal)
    sim.drawRobot(robot)
    sim.drawPoint(start.cartesianParameters, fill = 'green')
    sim.drawPoint(goal.cartesianParameters, fill = 'blue')


def drawProblemAndWait(sim, robot, obstacles, start, goal):
    drawRobotStartGoalObstacles(sim, robot, obstacles, start, goal)
    raw_input()