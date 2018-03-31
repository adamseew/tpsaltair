#!/bin/bash

echo "Compiling..."
g++ program.cpp src/vectorn.cpp src/integrator_rkn.cpp src/integrator_rk4.cpp src/wooden_ball.cpp src/cannon_ball.cpp src/solver_shooting.cpp -std=c++11

OPTION=$(whiptail --title "Test Menu Dialog" --menu "Choose your option" 15 60 4 \
    "1" "Wooden ball" \
    "2" "Cannon ball" \
    "3" "Shooting method"  3>&2 2>&1 1>&3	
)

exitstatus=$?
if [ $exitstatus = 0 ]; then
    case $OPTION in
        1)
            whiptail --title "Wooden ball" --msgbox "Simple wooden ball example with y axis only and varying velocity" 10 60
            echo "Running wooden ball..."
            echo "Saving data..."
            for i in {45..120..5}
            do
                ./a.out 0 $i > .tmp_$i.dat
            done
            ;;
        2)
            whiptail --title "Cannon ball" --msgbox "Simple cannon ball example with x,y axis and varying angle of fire" 10 60
            echo "Running cannon ball..."
            echo "Saving data..."
            for i in {25..95..5}
            do
                ./a.out 1 $i > .tmp_$i.dat
            done
            ;;
        3)
            whiptail --title "Shooting method" --msgbox "A shooting method solver using modified bisection method to adjust parameters" 10 60
            echo "Running shooting method on cannon ball..."
            echo "Saving data..."
            ./a.out 2 45
            ;;
    esac
    
    echo "Starting matlab..."
    echo -e "\e[1mType 'quit' to exit matlab\e[0m"
    matlab -nodesktop -nosplash -r "programtype=$OPTION;run plot_program.m;"
fi

echo "Cleaning..."
rm .tmp_*
rm a.out
echo "Done!"
