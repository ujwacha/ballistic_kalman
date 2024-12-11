#!/bin/bash
g++ -I/usr/include/eigen3/ simulation.cpp  && ./a.out > data.txt && gnuplot plotsctipt.gp
