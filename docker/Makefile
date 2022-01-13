allDistanceCsv := $(shell ls Distance*.csv | grep -v Merged)
allDistancePng := $(subst csv,png,$(allDistanceCsv))

allMetricsCsv := $(shell ls Metrics*.csv | grep -v Merged)
allMetricsPng := $(subst csv,png,$(allMetricsCsv))

allStateCsv := $(shell ls State*.csv | grep -v Merged)
allStatePng := $(subst csv,png,$(allStateCsv))

all-distance: $(allDistancePng)
all-metrics: $(allMetricsPng)
all-state: $(allStatePng)
all-trace: TraceXY.png TraceXZ.png TraceYZ.png

Distance%.png: Distance%.csv
	echo " \
		set datafile separator ',' \n\
		 \n\
		set term pngcairo size 1024,768 \n\
		set grid \n\
		set xlabel \"Time [s]\" \n\
		set ylabel \"Drone $< [m]\" \n\
		set yrange [0:3] \n\
		set output \"$@\" \n\
		plot     \"$<\" using (column(1)+column(2)/1000000000):(column(3)/32768) with lines title 'enable/established', \
						 \"$<\" using (column(1)+column(2)/1000000000):4 with lines title 'dist to 0', \
						 \"$<\" using (column(1)+column(2)/1000000000):5 with lines title 'dist to 1', \
				   	 \"$<\" using (column(1)+column(2)/1000000000):6 with lines title 'dist to 2', \
				   	 \"$<\" using (column(1)+column(2)/1000000000):7 with lines title 'dist to 3', \
				   	 \"$<\" using (column(1)+column(2)/1000000000):8 with lines title 'dist to 4', \
				   	 \"$<\" using (column(1)+column(2)/1000000000):9 with lines title 'dist to 5', \
				   	 \"$<\" using (column(1)+column(2)/1000000000):10 with lines title 'dist to 6', \
				   	 \"$<\" using (column(1)+column(2)/1000000000):11 with lines title 'dist to 7', \
				   	 \"$<\" using (column(1)+column(2)/1000000000):12 with lines title 'dist to 8', \
				   	 \"$<\" using (column(1)+column(2)/1000000000):13 with lines title 'dist to 9', \
				   	 \"$<\" using (column(1)+column(2)/1000000000):14 with lines title 'dist to 10', \
				   	 \"$<\" using (column(1)+column(2)/1000000000):15 with lines title 'dist to 11', \
				   	 \"$<\" using (column(1)+column(2)/1000000000):16 with lines title 'dist to 12', \
				   	 \"$<\" using (column(1)+column(2)/1000000000):17 with lines title 'dist to 13', \
				   	 \"$<\" using (column(1)+column(2)/1000000000):18 with lines title 'dist to 14'" \
	 | gnuplot

Metrics%.png: Metrics%.csv
	echo " \
		set datafile separator ',' \n\
		 \n\
	  stats \"$<\" using ((column(3)>64) ? column(4) : NaN) \n\
		print sprintf(\"metric: MEAN of dist min %f\", STATS_mean) \n\
		print sprintf(\"metric: MEDIAN of dist min %f\", STATS_median) \n\
		print sprintf(\"metric: MIN of dist min %f\", STATS_min) \n\
		print sprintf(\"metric: MAX of dist min %f\", STATS_max) \n\
		stats \"$<\" using ((column(3)>64) ? column(5) : NaN) \n\
		print sprintf(\"metric: MEAN of swarm radius %f\", STATS_mean) \n\
		print sprintf(\"metric: MEDIAN of swarm radius %f\", STATS_median) \n\
		print sprintf(\"metric: MIN of swarm radius %f\", STATS_min) \n\
		print sprintf(\"metric: MAX of swarm radius %f\", STATS_max) \n\
		stats \"$<\" using ((column(3)>64) ? column(6) : NaN) \n\
		print sprintf(\"metric: MEAN of dist obstacle min %f\", STATS_mean) \n\
		print sprintf(\"metric: MEDIAN of dist obstacle min %f\", STATS_median) \n\
		print sprintf(\"metric: MIN of dist obstacle min %f\", STATS_min) \n\
		print sprintf(\"metric: MAX of dist obstacle min %f\", STATS_max) \n\
		stats \"$<\" using ((column(3)>64) ? column(8) : NaN) \n\
		print sprintf(\"metric: MEAN of neighbours count %f\", STATS_mean) \n\
		print sprintf(\"metric: MEDIAN of neighbours count %f\", STATS_median) \n\
		print sprintf(\"metric: MIN of neighbours count %f\", STATS_min) \n\
		print sprintf(\"metric: MAX of neighbours count %f\", STATS_max) \n\
 		 \n\
		set term pngcairo size 1024,768 \n\
		set grid \n\
		set xlabel \"Time [s]\" \n\
		set ylabel \"Drone $< [m]\" \n\
		set yrange [0:3] \n\
		set output \"$@\" \n\
		plot     \"$<\" using (column(1)+column(2)/1000000000):(column(3)/32768) with lines title 'enable/established', \
						 \"$<\" using (column(1)+column(2)/1000000000):4 with lines title 'dist min', \
						 \"$<\" using (column(1)+column(2)/1000000000):5 with lines title 'swarm radius', \
						 \"$<\" using (column(1)+column(2)/1000000000):6 with lines title 'dist obstacle min', \
						 \"$<\" using (column(1)+column(2)/1000000000):7 with lines title 'drone count', \
						 \"$<\" using (column(1)+column(2)/1000000000):8 with lines title 'neighbours count'" \
	 | gnuplot

State%.png: State%.csv
	echo " \
 		set datafile separator ',' \n\
	  \n\
 		set term pngcairo size 1024,768 \n\
 		set grid \n\
 		set xlabel \"Time [s]\" \n\
 		set ylabel \"Position $< [m]\" \n\
		set y2label \"Velocity $< [m/s]\" \n\
		set xtics nomirror \n\
		set y2tics nomirror \n\
 		set output \"$@\" \n\
 		plot    \"$<\" using (column(1)+column(2)/1000000000):(column(3)/32768) with lines title 'enable/established', \
						\"$<\" using (column(1)+column(2)/1000000000):4 with lines title 'pos_x', \
						\"$<\" using (column(1)+column(2)/1000000000):5 with lines title 'pos_y', \
						\"$<\" using (column(1)+column(2)/1000000000):6 with lines title 'pos_z'" \
 	 | gnuplot

TraceXY.png: State0.csv Makefile
	echo " \
  		set datafile separator ',' \n\
  		 \n\
  		set term pngcairo size 1024,768 \n\
  		set grid \n\
  		set xlabel \"Position y [m]\" \n\
  		set ylabel \"Position x [m]\" \n\
			set yrange [-1:2] \n\
			set xrange [-2.5:3.5] \n\
  		set output \"$@\" \n\
  		plot  \"State0.csv\" using 5:((column(3)>1) ? column(4) : NaN) with lines title 'drone 0', \
						\"State1.csv\" using 5:((column(3)>1) ? column(4) : NaN) with lines title 'drone 1', \
						\"State2.csv\" using 5:((column(3)>1) ? column(4) : NaN) with lines title 'drone 2', \
						\"State3.csv\" using 5:((column(3)>1) ? column(4) : NaN) with lines title 'drone 3', \
						\"State4.csv\" using 5:((column(3)>1) ? column(4) : NaN) with lines title 'drone 4', \
						\"State5.csv\" using 5:((column(3)>1) ? column(4) : NaN) with lines title 'drone 5', \
						\"State6.csv\" using 5:((column(3)>1) ? column(4) : NaN) with lines title 'drone 6', \
						\"State7.csv\" using 5:((column(3)>1) ? column(4) : NaN) with lines title 'drone 7', \
						\"State8.csv\" using 5:((column(3)>1) ? column(4) : NaN) with lines title 'drone 8'" \
  	 | gnuplot

TraceXZ.png: State0.csv Makefile
	echo " \
		set datafile separator ',' \n\
		 \n\
		set term pngcairo size 1024,768 \n\
		set grid \n\
		set xlabel \"Position x [m]\" \n\
		set ylabel \"Position z [m]\" \n\
			set xrange [-2.5:3.5] \n\
			set yrange [-0.5:2.5] \n\
		set output \"$@\" \n\
		plot    \"State0.csv\" using 4:((column(3)>1) ? column(6) : NaN) with lines title 'drone 0', \
						\"State1.csv\" using 4:((column(3)>1) ? column(6) : NaN) with lines title 'drone 1', \
						\"State2.csv\" using 4:((column(3)>1) ? column(6) : NaN) with lines title 'drone 2', \
						\"State3.csv\" using 4:((column(3)>1) ? column(6) : NaN) with lines title 'drone 3', \
						\"State4.csv\" using 4:((column(3)>1) ? column(6) : NaN) with lines title 'drone 4', \
						\"State5.csv\" using 4:((column(3)>1) ? column(6) : NaN) with lines title 'drone 5', \
						\"State6.csv\" using 4:((column(3)>1) ? column(6) : NaN) with lines title 'drone 6', \
						\"State7.csv\" using 4:((column(3)>1) ? column(6) : NaN) with lines title 'drone 7', \
						\"State8.csv\" using 4:((column(3)>1) ? column(6) : NaN) with lines title 'drone 8'" \
	 | gnuplot


TraceYZ.png: State0.csv Makefile
	echo " \
 		set datafile separator ',' \n\
 		 \n\
 		set term pngcairo size 1024,768 \n\
 		set grid \n\
 		set xlabel \"Position y [m]\" \n\
 		set ylabel \"Position z [m]\" \n\
			set xrange [-2.5:3.5] \n\
			set yrange [-0.5:2.5] \n\
 		set output \"$@\" \n\
 		plot    \"State0.csv\" using 5:((column(3)>1) ? column(6) : NaN) with lines title 'drone 0', \
						\"State1.csv\" using 5:((column(3)>1) ? column(6) : NaN) with lines title 'drone 1', \
						\"State2.csv\" using 5:((column(3)>1) ? column(6) : NaN) with lines title 'drone 2', \
						\"State3.csv\" using 5:((column(3)>1) ? column(6) : NaN) with lines title 'drone 3', \
						\"State4.csv\" using 5:((column(3)>1) ? column(6) : NaN) with lines title 'drone 4', \
						\"State5.csv\" using 5:((column(3)>1) ? column(6) : NaN) with lines title 'drone 5', \
						\"State6.csv\" using 5:((column(3)>1) ? column(6) : NaN) with lines title 'drone 6', \
						\"State7.csv\" using 5:((column(3)>1) ? column(6) : NaN) with lines title 'drone 7', \
						\"State8.csv\" using 5:((column(3)>1) ? column(6) : NaN) with lines title 'drone 8'" \
 	 | gnuplot

CleanedMetrics%.csv: Metrics%.csv
	grep ",65," $< > $@
#	grep ",96," $< > $@
#	grep ",32," $< > $@

CleanedAllMetrics9.csv: CleanedMetrics0.csv CleanedMetrics1.csv CleanedMetrics2.csv CleanedMetrics3.csv CleanedMetrics4.csv CleanedMetrics5.csv CleanedMetrics6.csv CleanedMetrics7.csv CleanedMetrics8.csv Makefile
	paste CleanedMetrics0.csv CleanedMetrics1.csv CleanedMetrics2.csv CleanedMetrics3.csv CleanedMetrics4.csv CleanedMetrics5.csv CleanedMetrics6.csv CleanedMetrics7.csv CleanedMetrics8.csv |\
	awk -F, '{print $$1","$$2",99,\
"$$4","$$5",\
"$$12","$$13",\
"$$20","$$21",\
"$$28","$$29",\
"$$36","$$37",\
"$$44","$$45",\
"$$52","$$53",\
"$$60","$$61",\
"$$68","$$69}' | grep -v ",," > $@

CleanedAllMetrics5.csv: CleanedMetrics0.csv CleanedMetrics1.csv CleanedMetrics2.csv CleanedMetrics3.csv CleanedMetrics4.csv Makefile
	paste CleanedMetrics0.csv CleanedMetrics1.csv CleanedMetrics2.csv CleanedMetrics3.csv CleanedMetrics4.csv |\
	awk -F, '{print $$1","$$2",99,\
"$$4","$$5",\
"$$12","$$13",\
"$$20","$$21",\
"$$28","$$29",\
"$$36","$$37}' | grep -v ",," > $@

CleanedAllMetrics2.csv: CleanedMetrics0.csv CleanedMetrics1.csv
	paste CleanedMetrics0.csv CleanedMetrics1.csv |\
	awk -F, '{print $$1","$$2",99,\
"$$4","$$5","$$6",\
"$$12","$$13","$$14}' | grep -v ",," > $@

CleanedAllMetrics9.png: CleanedAllMetrics9.csv
	echo " \
		set datafile separator ',' \n\
		min(a,b) = (a < b) ? a : b \n\
		max(a,b) = (a < b) ? b : a \n\
		set term pngcairo size 1024,768 \n\
		set grid \n\
		set xlabel \"Time [s]\" \n\
		set ylabel \"Distance $< [m]\" \n\
		set yrange [0:3.0] \n\
		set output \"$@\" \n\
		plot     \"$<\" using (column(1)+column(2)/1000000000):(min(column(4),min(column(6),min(column(8),min(column(10),min(column(12),min(column(14),min(column(16),min(column(18),column(20)))))))))) with lines title 'dist_{min}', \
						 \"$<\" using (column(1)+column(2)/1000000000):(max(column(5),max(column(7),max(column(9),max(column(11),max(column(13),max(column(15),max(column(17),max(column(19),column(21)))))))))) with lines title 'comp_{max}" \
	 | gnuplot

CleanedAllMetrics5.png: CleanedAllMetrics5.csv
	echo " \
 		set datafile separator ',' \n\
 		min(a,b) = (a < b) ? a : b \n\
 		max(a,b) = (a < b) ? b : a \n\
 		set term pngcairo size 1024,768 \n\
 		set grid \n\
 		set xlabel \"Time [s]\" \n\
 		set ylabel \"Distance $< [m]\" \n\
 		set yrange [0:3.0] \n\
 		set output \"$@\" \n\
 		plot     \"$<\" using (column(1)+column(2)/1000000000):(min(column(4),min(column(6),min(column(8),min(column(10),column(12)))))) with lines title 'dist_{min}', \
 						 \"$<\" using (column(1)+column(2)/1000000000):(max(column(5),max(column(7),max(column(9),max(column(11),column(13)))))) with lines title 'comp_{max}" \
 	 | gnuplot

CleanedAllMetrics2.png: CleanedAllMetrics2.csv
	echo " \
		set datafile separator ',' \n\
		min(a,b) = (a < b) ? a : b \n\
		max(a,b) = (a < b) ? b : a \n\
		set term pngcairo size 1024,768 \n\
		set grid \n\
		set xlabel \"Time [s]\" \n\
		set ylabel \"Distance $< [m]\" \n\
		set yrange [0:3.0] \n\
		set autoscale ymax \n\
		set output \"$@\" \n\
		plot     \"$<\" using (column(1)+column(2)/1000000000):(min(column(4),column(7))) with lines title 'dist_{min}', \
					   \"$<\" using (column(1)+column(2)/1000000000):(min(column(5),column(8))) with lines title 'comp_{max}', \
						 \"$<\" using (column(1)+column(2)/1000000000):(max(column(6),column(9))) with lines title 'clear_{obj}" \
	 | gnuplot

CleanedAllMetricsVerbose.png: CleanedAllMetrics.csv
	 	echo " \
	 		set datafile separator ',' \n\
	 		min(a,b) = (a < b) ? a : b \n\
	 		max(a,b) = (a < b) ? b : a \n\
	 		set term pngcairo size 1024,768 \n\
	 		set grid \n\
			set xlabel \"Time [s]\" \n\
			set ylabel \"Distance $< [m]\" \n\
			set yrange [0:1.5] \n\
	 		set output \"$@\" \n\
	 		plot     \"$<\" using (column(1)+column(2)/1000000000):4 with lines title 'dist_min of drone 0', \
	 						 \"$<\" using (column(1)+column(2)/1000000000):6 with lines title 'dist_min of drone 1', \
	 				   	 \"$<\" using (column(1)+column(2)/1000000000):8 with lines title 'dist_min of drone 2', \
	 				   	 \"$<\" using (column(1)+column(2)/1000000000):10 with lines title 'dist_min of drone 3', \
	 				   	 \"$<\" using (column(1)+column(2)/1000000000):12 with lines title 'dist_min of drone 4', \
	 				   	 \"$<\" using (column(1)+column(2)/1000000000):14 with lines title 'dist_min of drone 5', \
	 				   	 \"$<\" using (column(1)+column(2)/1000000000):16 with lines title 'dist_min of drone 6', \
	 				   	 \"$<\" using (column(1)+column(2)/1000000000):18 with lines title 'dist_min of drone 7', \
	 				   	 \"$<\" using (column(1)+column(2)/1000000000):20 with lines title 'dist_min of drone 8', \
	 						 \"$<\" using (column(1)+column(2)/1000000000):5 with lines title 'dist_center of drone 0', \
	 						 \"$<\" using (column(1)+column(2)/1000000000):7 with lines title 'dist_center of drone 1', \
	 				   	 \"$<\" using (column(1)+column(2)/1000000000):9 with lines title 'dist_center of drone 2', \
	 				   	 \"$<\" using (column(1)+column(2)/1000000000):11 with lines title 'dist_center of drone 3', \
	 				   	 \"$<\" using (column(1)+column(2)/1000000000):13 with lines title 'dist_center of drone 4', \
	 				   	 \"$<\" using (column(1)+column(2)/1000000000):15 with lines title 'dist_center of drone 5', \
	 				   	 \"$<\" using (column(1)+column(2)/1000000000):17 with lines title 'dist_center of drone 6', \
	 				   	 \"$<\" using (column(1)+column(2)/1000000000):19 with lines title 'dist_center of drone 7', \
	 				   	 \"$<\" using (column(1)+column(2)/1000000000):21 with lines title 'dist_center of drone 8', \
	 						 \"$<\" using (column(1)+column(2)/1000000000):(min(column(4),min(column(6),min(column(8),min(column(10),min(column(12),min(column(14),min(column(16),min(column(18),column(20)))))))))) with lines title 'min of dist_min', \
	 						 \"$<\" using (column(1)+column(2)/1000000000):(max(column(5),max(column(7),max(column(9),max(column(11),max(column(13),max(column(15),max(column(17),max(column(19),column(21)))))))))) with lines title 'max of dist_center" \
	 	 | gnuplot



MetricsMerged.csv: $(allMetricsCsv)
	rm -rf $@
	cat $? > $@

StateMerged.csv: $(allStateCsv)
	rm -rf $@
	cat $? > $@

rm-csv:
	rm -f *.csv

clean:
	rm -f *.png

clean-all: clean rm-csv




all-step: DronePosition.png DronePositionTrajectory.png DroneAttitude.png XeYe.png DeltaCommands.png OmegaCommand.png RCommand.png PQCommands.png AttiudeCommands.png PropellersVelocity.png XYcontroller.png Attiudecontroller.png

DronePosition.png: DronePosition0.csv
	echo " \
		set datafile separator ',' \n\
		 \n\
		set term pngcairo size 1024,768 \n\
		set grid \n\
		set xlabel \"Time [s]\" \n\
 		set ylabel \"Position $< [m] (given by ground-truth odometry)\" \n\
		set y2label \"Velocity $< [m/s] (given by ground-truth odometry)\" \n\
		set xtics nomirror \n\
		set y2tics nomirror \n\
		set output \"$@\" \n\
		plot     \"DronePosition0.csv\" using (column(7)+column(8)/1000000000):1 with lines title 'X-Pos', \
						 \"DronePosition0.csv\" using (column(7)+column(8)/1000000000):2 with lines title 'Y-Pos', \
						 \"DronePosition0.csv\" using (column(7)+column(8)/1000000000):3 with lines title 'Z-Pos', \
 						 \"DronePosition0.csv\" using (column(7)+column(8)/1000000000):4 with lines title 'X-Velocity' axis x2y2, \
						 \"DronePosition0.csv\" using (column(7)+column(8)/1000000000):5 with lines title 'Y-Velocity' axis x2y2, \
 						 \"DronePosition0.csv\" using (column(7)+column(8)/1000000000):6 with lines title 'Z-Velocity' axis x2y2" \
	 | gnuplot
#	 set xrange [20:25] \n\
#	 set x2range [20:25] \n\
#	 set yrange [7.8:8.6] \n\


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
		plot     \"DronePosition0.csv\" using (column(7)+column(8)/1000000000):1 with lines title 'X-Pos', \
			 \"DronePosition0.csv\" using (column(7)+column(8)/1000000000):2 with lines title 'Y-Pos', \
			 \"DronePosition0.csv\" using (column(7)+column(8)/1000000000):3 with lines title 'Z-Pos', \
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
