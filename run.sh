#!/bin/bash

echo "Compiling..."
g++ program.cpp src/vectorn.cpp src/integrator_rkn.cpp src/integrator_rk4.cpp src/wooden_ball.cpp src/cannon_ball.cpp src/solver_shooting.cpp  -std=c++11
#echo "Running wooden ball..."
#echo "Saving data..."
#for i in {45..120..5}
#do
#    ./a.out 0 $i > .tmp_$i.dat
#done
#echo "Starting matlab..."
#echo -e "\e[1mType 'quit' to exit matlab\e[0m"
#matlab -nodesktop -nosplash -r "run plot_program.m;"
#echo "Cleaning..."
#rm .tmp_*
#echo "Running cannon ball..."
#echo "Saving data..."
#for i in {25..95..5}
#do
#    ./a.out 1 $i > .tmp_$i.dat
#done
#echo "Starting matlab..."
#echo -e "\e[1mType 'quit' to exit matlab\e[0m"
#matlab -nodesktop -nosplash -r "run plot_program.m;"
#echo "Cleaning..."
#rm .tmp_*
echo "Running shooting method on cannon ball..."
echo "Saving data..."
./a.out 2 45
echo "Starting matlab..."
echo -e "\e[1mType 'quit' to exit matlab\e[0m"
matlab -nodesktop -nosplash -r "run plot_program.m;"
echo "Cleaning..."
rm .tmp_*
rm a.out
echo "Done!"