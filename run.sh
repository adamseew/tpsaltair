#!/bin/bash

echo "Compiling..."
g++ program.cpp src/vectorn.cpp src/integrator_rkn.cpp src/integrator_rk4.cpp src/wooden_ball.cpp src/cannon_ball.cpp src/solver_shooting.cpp -std=c++11

OPTION=$(whiptail --title "TpsAltair" --menu "Choose an example from the list" 14 40 3 \
    "1" "  Wooden ball  " \
    "2" "  Cannon ball  " \
    "3" "Shooting method"  3>&2 2>&1 1>&3	
)

exitstatus=$?
if [ $exitstatus = 0 ]; then
    case $OPTION in
        1)
            whiptail --title "Wooden ball" --msgbox "Example description: a simple example of a thrown wooden ball, which involves only y axis only with varous initial velocities. Matlab is used to plot results." 14 40
            echo "Running wooden ball..."
            echo "Saving data..."
            for i in {45..120..5}
            do
                ./a.out 0 $i > .tmp_$i.dat
            done
            ;;
        2)
            whiptail --title "Cannon ball" --msgbox "Example description: a simple cannon ball example with x,y axis and varying angles of fire. Either drag and initial velocity is considered to model the problem. Matlab is used to plot results." 14 40
            echo "Running cannon ball..."
            echo "Saving data..."
            for i in {25..95..5}
            do
                ./a.out 1 $i > .tmp_$i.dat
            done
            ;;
        3)
            whiptail --title "Shooting method" --msgbox "Example description: a complex shooting method solver example, that represents a system of a flying cannon projectile, that uses modified bisection method to adjust parameters. The same ODE of cannon ball example is considered. Matlab is used to plot results." 14 40

            OPTION2=$(whiptail --title "Shooting method precision" --menu "Choose a desired precision" 14 73 2 \
                "N" "    Regular precision (reach a goal distant 10^3 km whithin 500 m)   " \
                "E" "Extremely high precision (spots a target distant 10^3 km of size 1 m)"  3>&2 2>&1 1>&3	
            )

            echo "Running shooting method on cannon ball..."
            echo "Saving data..."
            case $OPTION2 in
                E)
                    echo "warning: the solver may takes several minutes!"
                    echo -e "\033[0;31mh=0.0001, epsilon=0.001\033[0m"

                    count=0
                    ./a.out 2 45 1 |
                    stdbuf -oL tr -s '\r' '\n' |
                    while read -r str
                    do
                        ((count++))
                        echo -e "\033[0;31m$str\033[0m ($count/20)"
                    done
                    ;;
                *)
                    echo -e "\033[0;31mh=0.01, epsilon=0.5\033[0m"

                    count=0
                    ./a.out 2 45 |
                    stdbuf -oL tr -s '\r' '\n' |
                    while read -r str
                    do
                        ((count++))
                        echo -e "\033[0;31m$str\033[0m ($count/10)"
                    done 
                    ;;
            esac
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
