all: DronePosition.png DronePositionTrajectory.png DroneAttitude.png XeYe.png DeltaCommands.png OmegaCommand.png RCommand.png PQCommands.png AttiudeCommands.png PropellersVelocity.png XYcontroller.png Attiudecontroller.png

DronePosition.png: DronePosition.csv
	echo " \
		set datafile separator ',' \n\
		 \n\
		set term pngcairo size 1024,768 \n\
		set grid \n\
		set xlabel \"Time [s]\" \n\
		 \n\
		set output \"$@\" \n\
		set ylabel \"DronePosition (given by ground-truth odometry)\" \n\
		plot     \"DronePosition.csv\" using (column(4)+column(5)/1000000000):1 with lines title 'X-Pos', \
			 \"DronePosition.csv\" using (column(4)+column(5)/1000000000):2 with lines title 'Y-Pos', \
			 \"DronePosition.csv\" using (column(4)+column(5)/1000000000):3 with lines title 'Z-Pos'" \
	 | gnuplot  

DronePositionTrajectory.png: DronePosition.csv Trajectory.csv
	echo " \
		set datafile separator ',' \n\
		 \n\
		set term pngcairo size 1024,768 \n\
		set grid \n\
		set xlabel \"Time [s]\" \n\
		 \n\
		set output \"$@\" \n\
		set ylabel \"DronePosition (given by ground-truth odometry) and Trajectory (set-point commands)\" \n\
		plot     \"DronePosition.csv\" using (column(4)+column(5)/1000000000):1 with lines title 'X-Pos', \
			 \"DronePosition.csv\" using (column(4)+column(5)/1000000000):2 with lines title 'Y-Pos', \
			 \"DronePosition.csv\" using (column(4)+column(5)/1000000000):3 with lines title 'Z-Pos', \
			 \"Trajectory.csv\" using (column(4)+column(5)/1000000000):1 with lines title 'X-command', \
		 	 \"Trajectory.csv\" using (column(4)+column(5)/1000000000):2 with lines title 'Y-command', \
		 	 \"Trajectory.csv\" using (column(4)+column(5)/1000000000):3 with lines title 'Z-command'" \
	 | gnuplot  

DroneAttitude.png: DroneAttitude.csv
	echo " \
		set datafile separator ',' \n\
		 \n\
		set term pngcairo size 1024,768 \n\
		set grid \n\
		set xlabel \"Time [s]\" \n\
		 \n\
		set output \"$@\" \n\
		set ylabel \"Drone attitude (given by ground-truth odometry (?))\" \n\
		plot     \"DroneAttitude.csv\" using (column(4)+column(5)/1000000000):1 with lines title 'roll', \
			 \"DroneAttitude.csv\" using (column(4)+column(5)/1000000000):2 with lines title 'pitch', \
			 \"DroneAttitude.csv\" using (column(4)+column(5)/1000000000):3 with lines title 'yaw'" \
	 | gnuplot  

XeYe.png: XeYe.csv
	echo " \
		set datafile separator ',' \n\
		 \n\
		set term pngcairo size 1024,768 \n\
		set grid \n\
		set xlabel \"Time [s]\" \n\
		 \n\
		set output \"$@\" \n\
		set ylabel \"Drone position and velocity errors (of XYController)\" \n\
		plot     \"XeYe.csv\" using (column(5)+column(6)/1000000000):1 with lines title 'X-error', \
			 \"XeYe.csv\" using (column(5)+column(6)/1000000000):2 with lines title 'Y-error', \
			 \"XeYe.csv\" using (column(5)+column(6)/1000000000):3 with lines title 'X-velocity-error', \
			 \"XeYe.csv\" using (column(5)+column(6)/1000000000):4 with lines title 'Y-velocity-error'" \
	 | gnuplot  

DeltaCommands.png: DeltaCommands.csv
	echo " \
		set datafile separator ',' \n\
		 \n\
		set term pngcairo size 1024,768 \n\
		set grid \n\
		set xlabel \"Time [s]\" \n\
		 \n\
		set output \"$@\" \n\
		set ylabel \"Delta Commands (of RateController)\" \n\
		plot     \"DeltaCommands.csv\" using (column(4)+column(5)/1000000000):1 with lines title 'delta-phi', \
			 \"DeltaCommands.csv\" using (column(4)+column(5)/1000000000):2 with lines title 'delta-theta', \
			 \"DeltaCommands.csv\" using (column(4)+column(5)/1000000000):3 with lines title 'delta-psi'" \
	 | gnuplot  

OmegaCommand.png: OmegaCommand.csv
	echo " \
		set datafile separator ',' \n\
		 \n\
		set term pngcairo size 1024,768 \n\
		set grid \n\
		set xlabel \"Time [s]\" \n\
		 \n\
		set output \"$@\" \n\
		set ylabel \"Omega Command (of HoveringController)\" \n\
		plot     \"OmegaCommand.csv\" using (column(2)+column(3)/1000000000):1 with lines title 'omega-command'" \
	 | gnuplot  

RCommand.png: RCommand.csv
	echo " \
		set datafile separator ',' \n\
		 \n\
		set term pngcairo size 1024,768 \n\
		set grid \n\
		set xlabel \"Time [s]\" \n\
		 \n\
		set output \"$@\" \n\
		set ylabel \"R Command (of YawMpcController)\" \n\
		plot     \"RCommand.csv\" using (column(2)+column(3)/1000000000):1 with lines title 'r-command'" \
	 | gnuplot  

PQCommands.png: PQCommands.csv
	echo " \
		set datafile separator ',' \n\
		 \n\
		set term pngcairo size 1024,768 \n\
		set grid \n\
		set xlabel \"Time [s]\" \n\
		 \n\
		set output \"$@\" \n\
		set ylabel \"PQ Commands (of AttitudeController)\" \n\
		plot     \"PQCommands.csv\" using (column(5)+column(6)/1000000000):1 with lines title 'p-command', \
	 		\"PQCommands.csv\" using (column(5)+column(6)/1000000000):2 with lines title 'q-command'" \
	 | gnuplot  

AttiudeCommands.png: PQCommands.csv DroneAttitude.csv DronePosition.csv
	echo " \
		set datafile separator ',' \n\
		 \n\
		set term pngcairo size 1024,768 \n\
		set grid \n\
		set xlabel \"Time [s]\" \n\
		 \n\
		set output \"$@\" \n\
		set ylabel \"Attitude and Commands\" \n\
		plot     \"PQCommands.csv\" using (column(5)+column(6)/1000000000):3 with lines title 'phi-command', \
			 \"PQCommands.csv\" using (column(5)+column(6)/1000000000):4 with lines title 'theta-command', \
			 \"DroneAttitude.csv\" using (column(4)+column(5)/1000000000):2 with lines title 'pitch', \
			 \"DroneAttitude.csv\" using (column(4)+column(5)/1000000000):1 with lines title 'roll', \
			 \"DronePosition.csv\" using (column(4)+column(5)/1000000000):1 with lines title 'X-Pos', \
			 \"DronePosition.csv\" using (column(4)+column(5)/1000000000):2 with lines title 'Y-Pos'" \
	 | gnuplot  

PropellersVelocity.png: PropellersVelocity.csv
	echo " \
		set datafile separator ',' \n\
		 \n\
		set term pngcairo size 1024,768 \n\
		set grid \n\
		set xlabel \"Time [s]\" \n\
		 \n\
		set output \"$@\" \n\
		set ylabel \"Propellers velocity\" \n\
		plot     \"PropellersVelocity.csv\" using (column(5)+column(6)/1000000000):1 with lines title 'A', \
			 \"PropellersVelocity.csv\" using (column(5)+column(6)/1000000000):2 with lines title 'B', \
			 \"PropellersVelocity.csv\" using (column(5)+column(6)/1000000000):3 with lines title 'C', \
			 \"PropellersVelocity.csv\" using (column(5)+column(6)/1000000000):4 with lines title 'D'" \
	 | gnuplot  

XYcontroller.png: XeYe.csv CommandAttitude.csv
	echo " \
		set datafile separator ',' \n\
		 \n\
		set term pngcairo size 1024,768 \n\
		set grid \n\
		set xlabel \"Time [s]\" \n\
		 \n\
		set output \"$@\" \n\
		set ylabel \"XYcontroller\" \n\
		plot     \"XeYe.csv\" using (column(5)+column(6)/1000000000):1 with lines title 'error-x', \
			 \"XeYe.csv\" using (column(5)+column(6)/1000000000):2 with lines title 'error-y', \
			 \"XeYe.csv\" using (column(5)+column(6)/1000000000):3 with lines title 'error-Vx', \
			 \"XeYe.csv\" using (column(5)+column(6)/1000000000):4 with lines title 'error-Vy', \
			 \"CommandAttitude.csv\" using (column(3)+column(4)/1000000000):1 with lines title 'theta-command', \
			 \"CommandAttitude.csv\" using (column(3)+column(4)/1000000000):2 with lines title 'phi-command'" \
	 | gnuplot  
	 
Attiudecontroller.png: CommandAttitude.csv DroneAttitude.csv PQCommands.csv
	echo " \
		set datafile separator ',' \n\
		 \n\
		set term pngcairo size 1024,768 \n\
		set grid \n\
		set xlabel \"Time [s]\" \n\
		 \n\
		set output \"$@\" \n\
		set ylabel \"Attiudecontroller\" \n\
		plot     \"CommandAttitude.csv\" using (column(3)+column(4)/1000000000):1 with lines title 'theta-command', \
		   	 \"CommandAttitude.csv\" using (column(3)+column(4)/1000000000):2 with lines title 'phi-command', \
			 \"DroneAttitude.csv\" using (column(4)+column(5)/1000000000):2 with lines title 'pitch (theta)', \
			 \"DroneAttitude.csv\" using (column(4)+column(5)/1000000000):1 with lines title 'roll (phi)', \
		 	 \"PQCommands.csv\" using (column(5)+column(6)/1000000000):2 with lines title 'q-command (theta-command)', \
			 \"PQCommands.csv\" using (column(5)+column(6)/1000000000):1 with lines title 'p-command (phi-command)'" \
	 | gnuplot  


rm-csv:
	rm -f *.csv

clean:
	rm -f *.png
	
clean-all: clean rm-csv


