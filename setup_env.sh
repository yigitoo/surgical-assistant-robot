#!/bin/sh

pacman -S python-virtualenv
python3 -m venv ./venv
. ./venv/bin/activate
pip3 install -r requirements.txt
