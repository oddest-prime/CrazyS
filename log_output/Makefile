all: DronePosition.png DronePositionTrajectory.png DroneAttitude.png XeYe.png DeltaCommands.png OmegaCommand.png RCommand.png PQCommands.png AttiudeCommands.png PropellersVelocity.png XYcontroller.png Attiudecontroller.png

DronePosition.png: DronePosition0.csv
	echo " \
		set datafile separator ',' \n\
		 \n\
		set term pngcairo size 1024,768 \n\
		set grid \n\
		set xlabel \"Time [s]\" \n\
		 \n\
		set output \"$@\" \n\
		set ylabel \"DronePosition (given by ground-truth odometry)\" \n\
		plot     \"DronePosition0.csv\" using (column(4)+column(5)/1000000000):1 with lines title 'X-Pos', \
			 \"DronePosition0.csv\" using (column(4)+column(5)/1000000000):2 with lines title 'Y-Pos', \
			 \"DronePosition0.csv\" using (column(4)+column(5)/1000000000):3 with lines title 'Z-Pos'" \
	 | gnuplot  

DronePositionTrajectory.png: DronePosition0.csv Trajectory0.csv
	echo " \
		set datafile separator ',' \n\
		 \n\
		set term pngcairo size 1024,768 \n\
		set grid \n\
		set xlabel \"Time [s]\" \n\
		 \n\
		set output \"$@\" \n\
		set ylabel \"DronePosition (given by ground-truth odometry) and Trajectory (set-point commands)\" \n\
		plot     \"DronePosition0.csv\" using (column(4)+column(5)/1000000000):1 with lines title 'X-Pos', \
			 \"DronePosition0.csv\" using (column(4)+column(5)/1000000000):2 with lines title 'Y-Pos', \
			 \"DronePosition0.csv\" using (column(4)+column(5)/1000000000):3 with lines title 'Z-Pos', \
			 \"Trajectory0.csv\" using (column(4)+column(5)/1000000000):1 with lines title 'X-command', \
		 	 \"Trajectory0.csv\" using (column(4)+column(5)/1000000000):2 with lines title 'Y-command', \
		 	 \"Trajectory0.csv\" using (column(4)+column(5)/1000000000):3 with lines title 'Z-command'" \
	 | gnuplot  

DroneAttitude.png: DroneAttitude0.csv
	echo " \
		set datafile separator ',' \n\
		 \n\
		set term pngcairo size 1024,768 \n\
		set grid \n\
		set xlabel \"Time [s]\" \n\
		 \n\
		set output \"$@\" \n\
		set ylabel \"Drone attitude (given by ground-truth odometry (?))\" \n\
		plot     \"DroneAttitude0.csv\" using (column(4)+column(5)/1000000000):1 with lines title 'roll', \
			 \"DroneAttitude0.csv\" using (column(4)+column(5)/1000000000):2 with lines title 'pitch', \
			 \"DroneAttitude0.csv\" using (column(4)+column(5)/1000000000):3 with lines title 'yaw'" \
	 | gnuplot  

XeYe.png: XeYe0.csv
	echo " \
		set datafile separator ',' \n\
		 \n\
		set term pngcairo size 1024,768 \n\
		set grid \n\
		set xlabel \"Time [s]\" \n\
		 \n\
		set output \"$@\" \n\
		set ylabel \"Drone position and velocity errors (of XYController)\" \n\
		plot     \"XeYe0.csv\" using (column(5)+column(6)/1000000000):1 with lines title 'X-error', \
			 \"XeYe0.csv\" using (column(5)+column(6)/1000000000):2 with lines title 'Y-error', \
			 \"XeYe0.csv\" using (column(5)+column(6)/1000000000):3 with lines title 'X-velocity-error', \
			 \"XeYe0.csv\" using (column(5)+column(6)/1000000000):4 with lines title 'Y-velocity-error'" \
	 | gnuplot  

DeltaCommands.png: DeltaCommands0.csv
	echo " \
		set datafile separator ',' \n\
		 \n\
		set term pngcairo size 1024,768 \n\
		set grid \n\
		set xlabel \"Time [s]\" \n\
		 \n\
		set output \"$@\" \n\
		set ylabel \"Delta Commands (of RateController)\" \n\
		plot     \"DeltaCommands0.csv\" using (column(4)+column(5)/1000000000):1 with lines title 'delta-phi', \
			 \"DeltaCommands0.csv\" using (column(4)+column(5)/1000000000):2 with lines title 'delta-theta', \
			 \"DeltaCommands0.csv\" using (column(4)+column(5)/1000000000):3 with lines title 'delta-psi'" \
	 | gnuplot  

OmegaCommand.png: OmegaCommand0.csv
	echo " \
		set datafile separator ',' \n\
		 \n\
		set term pngcairo size 1024,768 \n\
		set grid \n\
		set xlabel \"Time [s]\" \n\
		 \n\
		set output \"$@\" \n\
		set ylabel \"Omega Command (of HoveringController)\" \n\
		plot     \"OmegaCommand0.csv\" using (column(2)+column(3)/1000000000):1 with lines title 'omega-command'" \
	 | gnuplot  

RCommand.png: RCommand0.csv
	echo " \
		set datafile separator ',' \n\
		 \n\
		set term pngcairo size 1024,768 \n\
		set grid \n\
		set xlabel \"Time [s]\" \n\
		 \n\
		set output \"$@\" \n\
		set ylabel \"R Command (of YawMpcController)\" \n\
		plot     \"RCommand0.csv\" using (column(2)+column(3)/1000000000):1 with lines title 'r-command'" \
	 | gnuplot  

PQCommands.png: PQCommands0.csv
	echo " \
		set datafile separator ',' \n\
		 \n\
		set term pngcairo size 1024,768 \n\
		set grid \n\
		set xlabel \"Time [s]\" \n\
		 \n\
		set output \"$@\" \n\
		set ylabel \"PQ Commands (of AttitudeController)\" \n\
		plot     \"PQCommands0.csv\" using (column(5)+column(6)/1000000000):1 with lines title 'p-command', \
	 		\"PQCommands0.csv\" using (column(5)+column(6)/1000000000):2 with lines title 'q-command'" \
	 | gnuplot  

AttiudeCommands.png: PQCommands0.csv DroneAttitude0.csv DronePosition0.csv
	echo " \
		set datafile separator ',' \n\
		 \n\
		set term pngcairo size 1024,768 \n\
		set grid \n\
		set xlabel \"Time [s]\" \n\
		 \n\
		set output \"$@\" \n\
		set ylabel \"Attitude and Commands\" \n\
		plot     \"PQCommands0.csv\" using (column(5)+column(6)/1000000000):3 with lines title 'phi-command', \
			 \"PQCommands0.csv\" using (column(5)+column(6)/1000000000):4 with lines title 'theta-command', \
			 \"DroneAttitude0.csv\" using (column(4)+column(5)/1000000000):2 with lines title 'pitch', \
			 \"DroneAttitude0.csv\" using (column(4)+column(5)/1000000000):1 with lines title 'roll', \
			 \"DronePosition0.csv\" using (column(4)+column(5)/1000000000):1 with lines title 'X-Pos', \
			 \"DronePosition0.csv\" using (column(4)+column(5)/1000000000):2 with lines title 'Y-Pos'" \
	 | gnuplot  

PropellersVelocity.png: PropellersVelocity0.csv
	echo " \
		set datafile separator ',' \n\
		 \n\
		set term pngcairo size 1024,768 \n\
		set grid \n\
		set xlabel \"Time [s]\" \n\
		 \n\
		set output \"$@\" \n\
		set ylabel \"Propellers velocity\" \n\
		plot     \"PropellersVelocity0.csv\" using (column(5)+column(6)/1000000000):1 with lines title 'A', \
			 \"PropellersVelocity0.csv\" using (column(5)+column(6)/1000000000):2 with lines title 'B', \
			 \"PropellersVelocity0.csv\" using (column(5)+column(6)/1000000000):3 with lines title 'C', \
			 \"PropellersVelocity0.csv\" using (column(5)+column(6)/1000000000):4 with lines title 'D'" \
	 | gnuplot  

XYcontroller.png: XeYe0.csv CommandAttitude0.csv
	echo " \
		set datafile separator ',' \n\
		 \n\
		set term pngcairo size 1024,768 \n\
		set grid \n\
		set xlabel \"Time [s]\" \n\
		 \n\
		set output \"$@\" \n\
		set ylabel \"XYcontroller\" \n\
		plot     \"XeYe0.csv\" using (column(5)+column(6)/1000000000):1 with lines title 'error-x', \
			 \"XeYe0.csv\" using (column(5)+column(6)/1000000000):2 with lines title 'error-y', \
			 \"XeYe0.csv\" using (column(5)+column(6)/1000000000):3 with lines title 'error-Vx', \
			 \"XeYe0.csv\" using (column(5)+column(6)/1000000000):4 with lines title 'error-Vy', \
			 \"CommandAttitude0.csv\" using (column(3)+column(4)/1000000000):1 with lines title 'theta-command', \
			 \"CommandAttitude0.csv\" using (column(3)+column(4)/1000000000):2 with lines title 'phi-command'" \
	 | gnuplot  
	 
Attiudecontroller.png: CommandAttitude0.csv DroneAttitude0.csv PQCommands0.csv
	echo " \
		set datafile separator ',' \n\
		 \n\
		set term pngcairo size 1024,768 \n\
		set grid \n\
		set xlabel \"Time [s]\" \n\
		 \n\
		set output \"$@\" \n\
		set ylabel \"Attiudecontroller\" \n\
		plot     \"CommandAttitude0.csv\" using (column(3)+column(4)/1000000000):1 with lines title 'theta-command', \
		   	 \"CommandAttitude0.csv\" using (column(3)+column(4)/1000000000):2 with lines title 'phi-command', \
			 \"DroneAttitude0.csv\" using (column(4)+column(5)/1000000000):2 with lines title 'pitch (theta)', \
			 \"DroneAttitude0.csv\" using (column(4)+column(5)/1000000000):1 with lines title 'roll (phi)', \
		 	 \"PQCommands0.csv\" using (column(5)+column(6)/1000000000):2 with lines title 'q-command (theta-command)', \
			 \"PQCommands0.csv\" using (column(5)+column(6)/1000000000):1 with lines title 'p-command (phi-command)'" \
	 | gnuplot  

Distance.png: Distance0.csv Distance1.csv Distance2.csv Distance3.csv
	echo " \
		set datafile separator ',' \n\
		 \n\
		set term pngcairo size 1024,768 \n\
		set grid \n\
		set xlabel \"Time [s]\" \n\
		 \n\
		set output \"$@\" \n\
		set ylabel \"Attiudecontroller\" \n\
		plot     \"Distance0.csv\" using (column(1)+column(2)/1000000000):3 with lines title 'dist 0-0', \
		   	 \"Distance0.csv\" using (column(1)+column(2)/1000000000):4 with lines title 'dist 0-1', \
		   	 \"Distance0.csv\" using (column(1)+column(2)/1000000000):5 with lines title 'dist 0-2', \
		   	 \"Distance0.csv\" using (column(1)+column(2)/1000000000):6 with lines title 'dist 0-3', \
		   	 \"Distance1.csv\" using (column(1)+column(2)/1000000000):3 with lines title 'dist 1-0', \
		   	 \"Distance1.csv\" using (column(1)+column(2)/1000000000):4 with lines title 'dist 1-1', \
		   	 \"Distance1.csv\" using (column(1)+column(2)/1000000000):5 with lines title 'dist 1-2', \
		   	 \"Distance1.csv\" using (column(1)+column(2)/1000000000):6 with lines title 'dist 1-3', \
		   	 \"Distance2.csv\" using (column(1)+column(2)/1000000000):3 with lines title 'dist 2-0', \
		   	 \"Distance2.csv\" using (column(1)+column(2)/1000000000):4 with lines title 'dist 2-1', \
		   	 \"Distance2.csv\" using (column(1)+column(2)/1000000000):5 with lines title 'dist 2-2', \
		   	 \"Distance2.csv\" using (column(1)+column(2)/1000000000):6 with lines title 'dist 2-3', \
		   	 \"Distance3.csv\" using (column(1)+column(2)/1000000000):3 with lines title 'dist 3-0', \
		   	 \"Distance3.csv\" using (column(1)+column(2)/1000000000):4 with lines title 'dist 3-1', \
		   	 \"Distance3.csv\" using (column(1)+column(2)/1000000000):5 with lines title 'dist 3-2', \
		   	 \"Distance3.csv\" using (column(1)+column(2)/1000000000):6 with lines title 'dist 3-3'" \
	 | gnuplot  

rm-csv:
	rm -f *.csv

clean:
	rm -f *.png
	
clean-all: clean rm-csv


