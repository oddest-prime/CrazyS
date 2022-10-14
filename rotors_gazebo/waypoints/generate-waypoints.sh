#!/bin/bash

rm -f waypoints_swarm_*.txt

input="waypoints_swarm.csv"
while read -r line
do
	if echo "$line" | grep -q "header"
	then
		echo "omit header" > /dev/null;
	else 
		timing=`echo "$line" | cut -d "," -f 1`
		
		iter=1
		while test $iter -lt 10
		do
			indexx=$((2 + ($iter-1)*4))
			indexy=$((3 + ($iter-1)*4))
			indexz=$((4 + ($iter-1)*4))
			indexyaw=$((5 + ($iter-1)*4))
			
			drone_x=`echo "$line" | cut -d "," -f $indexx`
			drone_y=`echo "$line" | cut -d "," -f $indexy`
			drone_z=`echo "$line" | cut -d "," -f $indexz`
			drone_yaw=`echo "$line" | cut -d "," -f $indexyaw`
			echo "$timing $drone_x $drone_y $drone_z $drone_yaw" >> waypoints_swarm_$iter.txt
			iter=$(($iter + 1))
		done
	fi;
done < "$input"

echo "done."
