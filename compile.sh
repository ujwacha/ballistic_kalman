#!/bin/bash
g++ -I/usr/include/eigen3/ simulation.cpp  && ./a.out && gnuplot plotsctipt.gp
