allDistanceCsv := $(shell ls Distance*.csv)
allDistancePng := $(subst csv,png,$(allDistanceCsv))

allMetricsCsv := $(shell ls Metrics*.csv)
allMetricsPng := $(subst csv,png,$(allMetricsCsv))

allStateCsv := $(shell ls State*.csv)
allStatePng := $(subst csv,png,$(allStateCsv))

all-distance: $(allDistancePng)
all-metrics: $(allMetricsPng)
all-state: $(allStatePng)

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
		plot     \"$<\" using (column(1)+column(2)/1000000000):(column(3)/64) with lines title 'enable/established', \
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
		set term pngcairo size 1024,768 \n\
		set grid \n\
		set xlabel \"Time [s]\" \n\
		set ylabel \"Drone $< [m]\" \n\
		set yrange [0:3] \n\
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
		set output \"$@\" \n\
		plot     \"$<\" using (column(1)+column(2)/1000000000):(column(3)/64) with lines title 'enable/established', \
						 \"$<\" using (column(1)+column(2)/1000000000):4 with lines title 'dist min', \
						 \"$<\" using (column(1)+column(2)/1000000000):5 with lines title 'swarm radius'" \
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
 		stats \"$<\" using ((column(3)>64) ? column(10) : NaN) \n\
 		print sprintf(\"metric: MEAN of abs vel %f\", STATS_mean) \n\
 		print sprintf(\"metric: MEDIAN of abs vel %f\", STATS_median) \n\
 		print sprintf(\"metric: MIN of abs vel %f\", STATS_min) \n\
 		print sprintf(\"metric: MAX of abs vel %f\", STATS_max) \n\
 		set output \"$@\" \n\
 		plot    \"$<\" using (column(1)+column(2)/1000000000):(column(3)/64) with lines title 'enable/established', \
						\"$<\" using (column(1)+column(2)/1000000000):4 with lines title 'pos_x', \
						\"$<\" using (column(1)+column(2)/1000000000):5 with lines title 'pos_y', \
						\"$<\" using (column(1)+column(2)/1000000000):6 with lines title 'pos_z', \
						\"$<\" using (column(1)+column(2)/1000000000):7 with lines title 'vel_x' axis x2y2, \
						\"$<\" using (column(1)+column(2)/1000000000):8 with lines title 'vel_y' axis x2y2, \
						\"$<\" using (column(1)+column(2)/1000000000):9 with lines title 'vel_z' axis x2y2, \
 						\"$<\" using (column(1)+column(2)/1000000000):10 with lines title '|vel|' axis x2y2" \
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
