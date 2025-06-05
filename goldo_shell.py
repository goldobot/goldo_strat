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
def config_nucleo():
    global my_main_loop
    asyncio.run_coroutine_threadsafe(async_config_nucleo(),my_main_loop)

async def async_config_nucleo():
    global my_robot
    await my_robot.configNucleo()

@goldobot_shell.command()
@click.argument('name', type=str, required=False)
@click.argument('goal', type=int)
def test_ax12(name="",goal=0):
    global my_main_loop
    asyncio.run_coroutine_threadsafe(async_test_ax12(name,goal),my_main_loop)

@goldobot_shell.command()
@click.argument('name', type=str, required=True)
@click.argument('arg1', type=str, required=False)
@click.argument('arg2', type=str, required=False)
def execute_sequence(name="",arg1=None,arg2=None):
    global my_main_loop
    asyncio.run_coroutine_threadsafe(async_execute_sequence(name,arg1,arg2),my_main_loop)

async def async_execute_sequence(sequence,arg1=None,arg2=None):
    global my_robot
    if ((arg1!=None) and (arg2!=None)):
        await my_robot._sequences[sequence](arg1,arg2)
    elif (arg1!=None):
        await my_robot._sequences[sequence](arg1)
    else:
        await my_robot._sequences[sequence]()

@goldobot_shell.command()
@click.argument('addr', type=str, required=True)
@click.argument('data', type=str, required=True)
def fpga_write(addr,data):
    global my_main_loop
    asyncio.run_coroutine_threadsafe(async_fpga_write(int(addr,16),int(data,16)),my_main_loop)

async def async_fpga_write(addr,data):
    global my_robot
    await my_robot.commands.fpgaRegWrite(addr,data)

@goldobot_shell.command()
@click.argument('addr', type=str, required=True)
def fpga_read(addr):
    global my_main_loop
    asyncio.run_coroutine_threadsafe(async_fpga_read(int(addr,16)),my_main_loop)

async def async_fpga_read(addr):
    global my_robot
    data = await my_robot.commands.fpgaRegRead(addr)
    print ("  data = {:x}".format(data))

#@goldobot_shell.command()
#@click.argument('name', type=str)
#def status_ax12(name=""):
#    global my_main_loop
#    global my_robot
#    my_robot._sequences_globals['servos'].printStatus(name)

