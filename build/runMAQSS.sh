#!/usr/bin/env bash

rm *log

./PNav -f ../InputFile  &>Plog &
./CV -f ../InputFile  &>Clog &


ps
