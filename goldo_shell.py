import asyncio
import click
from click_shell import shell
from os import _exit
from sys import argv
from time import sleep

my_main_loop = None

def goldobot_shell_set_main_loop(_loop):
    global my_main_loop
    my_main_loop = _loop

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

