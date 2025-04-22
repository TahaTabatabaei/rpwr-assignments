#!/usr/bin/python3

import os
import sys


def talk_with_the_snakes():
    print("What do you desire? Gold? (g) Light? (l)")
    response = input()
    if response == "g":
        print("Of course... How much do you desire?")
        quantity = input()
        os.system("echo '" + str(quantity) + "of gold' >> ../../../backpack")
        print("Something moves in your backpack.")
    elif response == "l":
        print("Wise choice...")
        print("...but nothing happens.")
    else:
        print("...after a while you hear them again.")
        talk_with_the_snakes()


if __name__ == "__main__":
    print("The snakes talk to you...")
    talk_with_the_snakes()
