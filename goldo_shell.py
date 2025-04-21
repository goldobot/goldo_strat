import asyncio
import click
from click_shell import shell
from os import _exit
from sys import argv
from time import sleep

my_main_loop = None
my_robot = None

def goldobot_shell_set_main_loop(_loop):
    global my_main_loop
    my_main_loop = _loop

def goldobot_shell_set_robot(_robot):
    global my_robot
    my_robot = _robot

@shell(prompt='goldo-shell > ', intro='Shell to control the robot')
def goldobot_shell():
    pass

async def async_test():
    print ("TE..")
    await asyncio.sleep(10.0)
    print ("..ST!!..")

@goldobot_shell.command()
def test():
    global my_main_loop
    asyncio.run_coroutine_threadsafe(async_test(),my_main_loop)

async def async_test_ax12(name,goal):
    global my_robot
    #await my_robot._sequences_globals['servos'].autotest(name, goal)
    await my_robot._sequences_globals['servos'].setEnable(name, True)
    await my_robot._sequences_globals['servos'].move(name, goal)

@goldobot_shell.command()
@click.argument('name', type=str, required=False)
@click.argument('goal', type=int)
def test_ax12(name="",goal=0):
    global my_main_loop
    asyncio.run_coroutine_threadsafe(async_test_ax12(name,goal),my_main_loop)

#@goldobot_shell.command()
#@click.argument('name', type=str)
#def status_ax12(name=""):
#    global my_main_loop
#    global my_robot
#    my_robot._sequences_globals['servos'].printStatus(name)

