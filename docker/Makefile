allDistanceCsv := $(shell ls Distance*.csv | grep -v Merged)
allDistancePng := $(subst csv,png,$(allDistanceCsv))

allMetricsCsv := $(shell ls Metrics*.csv | grep -v Merged)
allMetricsPng := $(subst csv,png,$(allMetricsCsv))

allStateCsv := $(shell ls State*.csv | grep -v Merged)
allStatePng := $(subst csv,png,$(allStateCsv))
allNoisePng := $(subst State,Noise,$(allStatePng))

all-distance: $(allDistancePng)
all-metrics: $(allMetricsPng)
all-state: $(allStatePng)
all-noise: $(allNoisePng)
all-trace: TraceXY.pdf TraceXZ.pdf TraceYZ.pdf

0obstacles_17drones_alternative_logs_%.pdf: %.pdf
	mv $< $@

# plot of cleaned trajectory
plot_start_val := $(shell grep plot_start_val crazyflie2_*.yaml | cut -d ":" -f 2)
plot_end_val := $(shell grep plot_end_val crazyflie2_*.yaml | cut -d ":" -f 2)
# noise of localization while drones still resting on ground
noise_start_val := $(shell grep noise_start_val crazyflie2_*.yaml | cut -d ":" -f 2)
noise_end_val := $(shell grep noise_end_val crazyflie2_*.yaml | cut -d ":" -f 2)

CombinedMetrics.csv: metrics.py
	./metrics.py

CombinedMetrics.pdf: CombinedMetrics.csv Makefile
	echo " \
		set datafile separator ',' \n\
		set term pdf font \"Times,15\" size 10cm, 7cm \n\
		set grid \n\
		set xlabel \"Time [s]\" \n\
		set ylabel \"Drone $< [m]\" \n\
  		unset xlabel \n\
  		unset ylabel \n\
		set yrange [0:3] \n\
		set xrange [0:($(plot_end_val)-$(plot_start_val))] \n\
		set output \"$@\" \n\
  		plot \"$<\" using (column(1)-$(plot_start_val)):(column(1)>$(plot_start_val) ? (column(1)<$(plot_end_val) ? column(2) : NaN) : NaN) with lines title 'dist min', \
			 \"$<\" using (column(1)-$(plot_start_val)):(column(1)>$(plot_start_val) ? (column(1)<$(plot_end_val) ? column(3) : NaN) : NaN) with lines title 'swarm radius', \
			 \"$<\" using (column(1)-$(plot_start_val)):(column(1)>$(plot_start_val) ? (column(1)<$(plot_end_val) ? column(4) : NaN) : NaN) with lines title 'dist obstacle min', \
 			0.2 with lines linewidth 2 lt rgb 'light-red' dashtype 2 title 'dist_{thr} (20cm)', \
			0.3 with lines linewidth 2 lt rgb 'coral' dashtype 2 title 'clear_{thr} (30cm)'" \
	 | gnuplot

Distance%.png: Distance%.csv Makefile
	echo " \
		set datafile separator ',' \n\
		 \n\
		set term pngcairo size 1024,768 \n\
		set grid \n\
		set xlabel \"Time [s]\" \n\
		set ylabel \"Drone $< [m]\" \n\
		set yrange [0:3] \n\
		set output \"$@\" \n\
		plot     \"$<\" using (column(1)+column(2)/1000000000):(column(3)/4) with lines title 'enable/established', \
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
				   	 \"$<\" using (column(1)+column(2)/1000000000):18 with lines title 'dist to 14', \
				   	 \"$<\" using (column(1)+column(2)/1000000000):19 with lines title 'dist to 15', \
				   	 \"$<\" using (column(1)+column(2)/1000000000):20 with lines title 'dist to 16', \
				   	 \"$<\" using (column(1)+column(2)/1000000000):21 with lines title 'dist to 17', \
				   	 \"$<\" using (column(1)+column(2)/1000000000):22 with lines title 'dist to 18', \
				   	 \"$<\" using (column(1)+column(2)/1000000000):23 with lines title 'dist to 19'" \
	 | gnuplot

Metrics%.png: Metrics%.csv Makefile
	echo " \
		set datafile separator ',' \n\
		 \n\
	    stats \"$<\" using ((column(3)>1) ? column(4) : NaN) \n\
		print sprintf(\"metric: MEAN of dist min %f\", STATS_mean) \n\
		print sprintf(\"metric: MEDIAN of dist min %f\", STATS_median) \n\
		print sprintf(\"metric: MIN of dist min %f\", STATS_min) \n\
		print sprintf(\"metric: MAX of dist min %f\", STATS_max) \n\
		stats \"$<\" using ((column(3)>1) ? column(5) : NaN) \n\
		print sprintf(\"metric: MEAN of swarm radius %f\", STATS_mean) \n\
		print sprintf(\"metric: MEDIAN of swarm radius %f\", STATS_median) \n\
		print sprintf(\"metric: MIN of swarm radius %f\", STATS_min) \n\
		print sprintf(\"metric: MAX of swarm radius %f\", STATS_max) \n\
		stats \"$<\" using ((column(3)>1) ? column(6) : NaN) \n\
		print sprintf(\"metric: MEAN of dist obstacle min %f\", STATS_mean) \n\
		print sprintf(\"metric: MEDIAN of dist obstacle min %f\", STATS_median) \n\
		print sprintf(\"metric: MIN of dist obstacle min %f\", STATS_min) \n\
		print sprintf(\"metric: MAX of dist obstacle min %f\", STATS_max) \n\
		stats \"$<\" using ((column(3)>1) ? column(8) : NaN) \n\
		print sprintf(\"metric: MEAN of neighbours count %f\", STATS_mean) \n\
		print sprintf(\"metric: MEDIAN of neighbours count %f\", STATS_median) \n\
		print sprintf(\"metric: MIN of neighbours count %f\", STATS_min) \n\
		print sprintf(\"metric: MAX of neighbours count %f\", STATS_max) \n\
 		 \n\
		set term pngcairo size 1024,768 \n\
		set grid \n\
		set xlabel \"Time [s]\" \n\
		set ylabel \"Distance [m]\" \n\
		set y2label \"Count\" \n\
		set yrange [0:5] \n\
		set y2range [0:20] \n\
		set xtics nomirror \n\
		set y2tics nomirror \n\
		set output \"$@\" \n\
		plot     \"$<\" using (column(1)+column(2)/1000000000):(column(3)/4) with lines title 'enable/established' axes x1y2, \
						 \"$<\" using (column(1)+column(2)/1000000000):4 with lines title 'dist to oth drone min', \
						 \"$<\" using (column(1)+column(2)/1000000000):5 with lines title 'dist to center', \
						 \"$<\" using (column(1)+column(2)/1000000000):6 with lines title 'dist obstacle min', \
						 \"$<\" using (column(1)+column(2)/1000000000):7 with lines title 'dist to target', \
						 \"$<\" using (column(1)+column(2)/1000000000):8 with lines title 'drone count' axes x1y2, \
						 \"$<\" using (column(1)+column(2)/1000000000):9 with lines title 'neighbours count' axes x1y2" \
	 | gnuplot

State%.png: State%.csv Makefile
	echo " \
 		set datafile separator ',' \n\
	  \n\
 		set term pngcairo size 1024,768 \n\
 		set grid \n\
 		set xlabel \"Time [s]\" \n\
 		set ylabel \"Position $< [m]\" \n\
		set y2label \"Count\" \n\
		set y2range [0:20] \n\
		set xtics nomirror \n\
		set y2tics nomirror \n\
 		set output \"$@\" \n\
		set output \"$@\" \n\
		plot    \"$<\" using (column(1)+column(2)/1000000000):(column(3)/4) with lines title 'enable/established' axes x1y2, \
						\"$<\" using (column(1)+column(2)/1000000000):4 with lines title 'pos_x', \
						\"$<\" using (column(1)+column(2)/1000000000):5 with lines title 'pos_y', \
						\"$<\" using (column(1)+column(2)/1000000000):6 with lines title 'pos_z'" \
 	 | gnuplot

Noise%.png: State%.csv Makefile
	echo " \
 		set datafile separator ',' \n\
	  \n\
 		set term pngcairo size 1024,768 \n\
 		set grid \n\
 		set xlabel \"Time [s]\" \n\
 		set ylabel \"Position before start $< [m]\" \n\
		set xrange [0:($(noise_end_val)-$(noise_start_val))] \n\
 		set output \"$@\" \n\
	    stats \"$<\" using ((column(1)+column(2)/1000000000)-$(noise_start_val)):((column(1)+column(2)/1000000000)>$(noise_start_val) ? ((column(1)+column(2)/1000000000)<$(noise_end_val) ? column(4) : NaN) : NaN) name 'statsX' output \n\
	    stats \"$<\" using ((column(1)+column(2)/1000000000)-$(noise_start_val)):((column(1)+column(2)/1000000000)>$(noise_start_val) ? ((column(1)+column(2)/1000000000)<$(noise_end_val) ? column(5) : NaN) : NaN) name 'statsY' output \n\
	    stats \"$<\" using ((column(1)+column(2)/1000000000)-$(noise_start_val)):((column(1)+column(2)/1000000000)>$(noise_start_val) ? ((column(1)+column(2)/1000000000)<$(noise_end_val) ? column(6) : NaN) : NaN) name 'statsZ' output \n\
  		plot \"$<\" using ((column(1)+column(2)/1000000000)-$(noise_start_val)):((column(1)+column(2)/1000000000)>$(noise_start_val) ? ((column(1)+column(2)/1000000000)<$(noise_end_val) ? column(4) : NaN) : NaN) with lines title 'pos_x', \
  		     \"$<\" using ((column(1)+column(2)/1000000000)-$(noise_start_val)):((column(1)+column(2)/1000000000)>$(noise_start_val) ? ((column(1)+column(2)/1000000000)<$(noise_end_val) ? column(5) : NaN) : NaN) with lines title 'pos_y', \
             \"$<\" using ((column(1)+column(2)/1000000000)-$(noise_start_val)):((column(1)+column(2)/1000000000)>$(noise_start_val) ? ((column(1)+column(2)/1000000000)<$(noise_end_val) ? column(6) : NaN) : NaN) with lines title 'pos_z', \
             \"$<\" using ((column(1)+column(2)/1000000000)-$(noise_start_val)):((column(1)+column(2)/1000000000)>$(noise_start_val) ? ((column(1)+column(2)/1000000000)<$(noise_end_val) ? statsX_mean_y : NaN) : NaN) with lines title 'mean_x', \
             \"$<\" using ((column(1)+column(2)/1000000000)-$(noise_start_val)):((column(1)+column(2)/1000000000)>$(noise_start_val) ? ((column(1)+column(2)/1000000000)<$(noise_end_val) ? statsY_mean_y : NaN) : NaN) with lines title 'mean_y', \
             \"$<\" using ((column(1)+column(2)/1000000000)-$(noise_start_val)):((column(1)+column(2)/1000000000)>$(noise_start_val) ? ((column(1)+column(2)/1000000000)<$(noise_end_val) ? statsZ_mean_y : NaN) : NaN) with lines title 'mean_z', \
             \"$<\" using ((column(1)+column(2)/1000000000)-$(noise_start_val)):((column(1)+column(2)/1000000000)>$(noise_start_val) ? ((column(1)+column(2)/1000000000)<$(noise_end_val) ? statsX_mean_y+statsX_stddev_y : NaN) : NaN) with lines title 'mean_x+stddev_x', \
             \"$<\" using ((column(1)+column(2)/1000000000)-$(noise_start_val)):((column(1)+column(2)/1000000000)>$(noise_start_val) ? ((column(1)+column(2)/1000000000)<$(noise_end_val) ? statsY_mean_y+statsY_stddev_y : NaN) : NaN) with lines title 'mean_y+stddev_y', \
             \"$<\" using ((column(1)+column(2)/1000000000)-$(noise_start_val)):((column(1)+column(2)/1000000000)>$(noise_start_val) ? ((column(1)+column(2)/1000000000)<$(noise_end_val) ? statsZ_mean_y+statsZ_stddev_y : NaN) : NaN) with lines title 'mean_z+stddev_z'" \
 	 | gnuplot

TraceXY.pdf: State0.csv Makefile crazyflie2_*.yaml
	echo " \
  		set datafile separator ',' \n\
  		 \n\
		set term pdf font \"Times,15\" size 10cm, 7cm \n\
		set grid \n\
		v0 = 0 \n v1 = 0 \n v2 = 0 \n v3 = 0 \n v4 = 0 \n\
		v5 = 0 \n v6 = 0 \n v7 = 0 \n v8 = 0 \n v9 = 0 \n\
		v10 = 0 \n v11 = 0 \n v12 = 0 \n v13 = 0 \n v14 = 0 \n\
		v15 = 0 \n v16 = 0 \n v17 = 0 \n v18 = 0 \n v19 = 0 \n\
		set key off \n\
        set xtics 2 \n\
        set ytics 2 \n\
  		set xlabel \"Position y [m]\" \n\
  		set ylabel \"Position x [m]\" \n\
  		unset xlabel \n\
  		unset ylabel \n\
		set xrange [-1.0:14.0] \n\
		set yrange [-4.5:4.5] \n\
  		set output \"$@\" \n\
  		plot  		    \"State0.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(4) : NaN) : NaN) with lines linecolor rgb \"web-blue\" linewidth 1 title 'drone 0', \
						\"State1.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(4) : NaN) : NaN) with lines linecolor rgb \"web-blue\" linewidth 1 title 'drone 1', \
						\"State2.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(4) : NaN) : NaN) with lines linecolor rgb \"web-blue\" linewidth 1 title 'drone 2', \
						\"State3.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(4) : NaN) : NaN) with lines linecolor rgb \"web-blue\" linewidth 1 title 'drone 3', \
						\"State4.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(4) : NaN) : NaN) with lines linecolor rgb \"web-blue\" linewidth 1 title 'drone 4', \
						\"State5.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(4) : NaN) : NaN) with lines linecolor rgb \"web-blue\" linewidth 1 title 'drone 5', \
						\"State6.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(4) : NaN) : NaN) with lines linecolor rgb \"web-blue\" linewidth 1 title 'drone 6', \
						\"State7.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(4) : NaN) : NaN) with lines linecolor rgb \"web-blue\" linewidth 1 title 'drone 7', \
						\"State8.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(4) : NaN) : NaN) with lines linecolor rgb \"web-blue\" linewidth 1 title 'drone 8', \
						\"State9.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(4) : NaN) : NaN) with lines linecolor rgb \"web-blue\" linewidth 1 title 'drone 9', \
						\"State10.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(4) : NaN) : NaN) with lines linecolor rgb \"web-blue\" linewidth 1 title 'drone 10', \
						\"State11.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(4) : NaN) : NaN) with lines linecolor rgb \"web-blue\" linewidth 1 title 'drone 11', \
						\"State13.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(4) : NaN) : NaN) with lines linecolor rgb \"web-blue\" linewidth 1 title 'drone 13', \
						\"State12.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(4) : NaN) : NaN) with lines linecolor rgb \"web-blue\" linewidth 1 title 'drone 12', \
						\"State14.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(4) : NaN) : NaN) with lines linecolor rgb \"web-blue\" linewidth 1 title 'drone 14', \
						\"State15.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(4) : NaN) : NaN) with lines linecolor rgb \"web-blue\" linewidth 1 title 'drone 15', \
						\"State16.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(4) : NaN) : NaN) with lines linecolor rgb \"web-blue\" linewidth 1 title 'drone 16', \
						\"State0.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_end_val)) ? ((v0=v0+1) <= 1 ? column(4) : NaN) : NaN) with points linecolor rgb \"dark-blue\" pointtype 2 title 'drone 0', \
						\"State1.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_end_val)) ? ((v1=v1+1) <= 1 ? column(4) : NaN) : NaN) with points linecolor rgb \"dark-blue\" pointtype 2 title 'drone 1', \
						\"State2.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_end_val)) ? ((v2=v2+1) <= 1 ? column(4) : NaN) : NaN) with points linecolor rgb \"dark-blue\" pointtype 2 title 'drone 2', \
						\"State3.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_end_val)) ? ((v3=v3+1) <= 1 ? column(4) : NaN) : NaN) with points linecolor rgb \"dark-blue\" pointtype 2 title 'drone 3', \
						\"State4.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_end_val)) ? ((v4=v4+1) <= 1 ? column(4) : NaN) : NaN) with points linecolor rgb \"dark-blue\" pointtype 2 title 'drone 4', \
						\"State5.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_end_val)) ? ((v5=v5+1) <= 1 ? column(4) : NaN) : NaN) with points linecolor rgb \"dark-blue\" pointtype 2 title 'drone 5', \
						\"State6.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_end_val)) ? ((v6=v6+1) <= 1 ? column(4) : NaN) : NaN) with points linecolor rgb \"dark-blue\" pointtype 2 title 'drone 6', \
						\"State7.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_end_val)) ? ((v7=v7+1) <= 1 ? column(4) : NaN) : NaN) with points linecolor rgb \"dark-blue\" pointtype 2 title 'drone 7', \
						\"State8.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_end_val)) ? ((v8=v8+1) <= 1 ? column(4) : NaN) : NaN) with points linecolor rgb \"dark-blue\" pointtype 2 title 'drone 8', \
						\"State9.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_end_val)) ? ((v9=v9+1) <= 1 ? column(4) : NaN) : NaN) with points linecolor rgb \"dark-blue\" pointtype 2 title 'drone 9', \
						\"State10.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_end_val)) ? ((v10=v10+1) <= 1 ? column(4) : NaN) : NaN) with points linecolor rgb \"dark-blue\" pointtype 2 title 'drone 10', \
						\"State11.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_end_val)) ? ((v11=v11+1) <= 1 ? column(4) : NaN) : NaN) with points linecolor rgb \"dark-blue\" pointtype 2 title 'drone 11', \
						\"State12.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_end_val)) ? ((v12=v12+1) <= 1 ? column(4) : NaN) : NaN) with points linecolor rgb \"dark-blue\" pointtype 2 title 'drone 12', \
						\"State13.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_end_val)) ? ((v13=v13+1) <= 1 ? column(4) : NaN) : NaN) with points linecolor rgb \"dark-blue\" pointtype 2 title 'drone 13', \
						\"State14.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_end_val)) ? ((v14=v14+1) <= 1 ? column(4) : NaN) : NaN) with points linecolor rgb \"dark-blue\" pointtype 2 title 'drone 14', \
						\"State15.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_end_val)) ? ((v15=v15+1) <= 1 ? column(4) : NaN) : NaN) with points linecolor rgb \"dark-blue\" pointtype 2 title 'drone 15', \
						\"State16.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_end_val)) ? ((v16=v16+1) <= 1 ? column(4) : NaN) : NaN) with points linecolor rgb \"dark-blue\" pointtype 2 title 'drone 16', \
						\"State0.csv\" using 8:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(7) : NaN) : NaN) with lines linecolor rgb \"light-red\" linewidth 1 title 'TARGET'" \
  	 | gnuplot



TraceXZ.pdf: State0.csv Makefile crazyflie2_*.yaml
	echo " \
  		set datafile separator ',' \n\
  		 \n\
		set term pdf font \"Times,15\" size 10cm, 7cm \n\
		set grid \n\
		v0 = 0 \n v1 = 0 \n v2 = 0 \n v3 = 0 \n v4 = 0 \n\
		v5 = 0 \n v6 = 0 \n v7 = 0 \n v8 = 0 \n v9 = 0 \n\
		v10 = 0 \n v11 = 0 \n v12 = 0 \n v13 = 0 \n v14 = 0 \n\
		v15 = 0 \n v16 = 0 \n v17 = 0 \n v18 = 0 \n v19 = 0 \n\
		set key off \n\
        set xtics 2 \n\
        set ytics 2 \n\
  		set xlabel \"Position x [m]\" \n\
  		set ylabel \"Position z [m]\" \n\
  		unset xlabel \n\
  		unset ylabel \n\
		set xrange [-4.5:4.5] \n\
		set yrange [-0.5:8.0] \n\
  		set output \"$@\" \n\
	    plot  		    \"State0.csv\" using 4:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linecolor rgb \"web-blue\" linewidth 1 title 'drone 0', \
						\"State1.csv\" using 4:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linecolor rgb \"web-blue\" linewidth 1 title 'drone 1', \
						\"State2.csv\" using 4:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linecolor rgb \"web-blue\" linewidth 1 title 'drone 2', \
						\"State3.csv\" using 4:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linecolor rgb \"web-blue\" linewidth 1 title 'drone 3', \
						\"State4.csv\" using 4:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linecolor rgb \"web-blue\" linewidth 1 title 'drone 4', \
						\"State5.csv\" using 4:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linecolor rgb \"web-blue\" linewidth 1 title 'drone 5', \
						\"State6.csv\" using 4:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linecolor rgb \"web-blue\" linewidth 1 title 'drone 6', \
						\"State7.csv\" using 4:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linecolor rgb \"web-blue\" linewidth 1 title 'drone 7', \
						\"State8.csv\" using 4:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linecolor rgb \"web-blue\" linewidth 1 title 'drone 8', \
						\"State9.csv\" using 4:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linecolor rgb \"web-blue\" linewidth 1 title 'drone 9', \
						\"State10.csv\" using 4:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linecolor rgb \"web-blue\" linewidth 1 title 'drone 10', \
						\"State11.csv\" using 4:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linecolor rgb \"web-blue\" linewidth 1 title 'drone 11', \
						\"State12.csv\" using 4:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linecolor rgb \"web-blue\" linewidth 1 title 'drone 12', \
						\"State13.csv\" using 4:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linecolor rgb \"web-blue\" linewidth 1 title 'drone 13', \
						\"State14.csv\" using 4:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linecolor rgb \"web-blue\" linewidth 1 title 'drone 14', \
						\"State15.csv\" using 4:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linecolor rgb \"web-blue\" linewidth 1 title 'drone 15', \
						\"State16.csv\" using 4:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linecolor rgb \"web-blue\" linewidth 1 title 'drone 16', \
						\"State0.csv\" using 4:(((column(1)+column(2)/1000000000)>$(plot_end_val)) ? ((v0=v0+1) <= 1 ? column(6) : NaN) : NaN) with points linecolor rgb \"dark-blue\" pointtype 2 title 'drone 0', \
						\"State1.csv\" using 4:(((column(1)+column(2)/1000000000)>$(plot_end_val)) ? ((v1=v1+1) <= 1 ? column(6) : NaN) : NaN) with points linecolor rgb \"dark-blue\" pointtype 2 title 'drone 1', \
						\"State2.csv\" using 4:(((column(1)+column(2)/1000000000)>$(plot_end_val)) ? ((v2=v2+1) <= 1 ? column(6) : NaN) : NaN) with points linecolor rgb \"dark-blue\" pointtype 2 title 'drone 2', \
						\"State3.csv\" using 4:(((column(1)+column(2)/1000000000)>$(plot_end_val)) ? ((v3=v3+1) <= 1 ? column(6) : NaN) : NaN) with points linecolor rgb \"dark-blue\" pointtype 2 title 'drone 3', \
						\"State4.csv\" using 4:(((column(1)+column(2)/1000000000)>$(plot_end_val)) ? ((v4=v4+1) <= 1 ? column(6) : NaN) : NaN) with points linecolor rgb \"dark-blue\" pointtype 2 title 'drone 4', \
						\"State5.csv\" using 4:(((column(1)+column(2)/1000000000)>$(plot_end_val)) ? ((v5=v5+1) <= 1 ? column(6) : NaN) : NaN) with points linecolor rgb \"dark-blue\" pointtype 2 title 'drone 5', \
						\"State6.csv\" using 4:(((column(1)+column(2)/1000000000)>$(plot_end_val)) ? ((v6=v6+1) <= 1 ? column(6) : NaN) : NaN) with points linecolor rgb \"dark-blue\" pointtype 2 title 'drone 6', \
						\"State7.csv\" using 4:(((column(1)+column(2)/1000000000)>$(plot_end_val)) ? ((v7=v7+1) <= 1 ? column(6) : NaN) : NaN) with points linecolor rgb \"dark-blue\" pointtype 2 title 'drone 7', \
						\"State8.csv\" using 4:(((column(1)+column(2)/1000000000)>$(plot_end_val)) ? ((v8=v8+1) <= 1 ? column(6) : NaN) : NaN) with points linecolor rgb \"dark-blue\" pointtype 2 title 'drone 8', \
						\"State9.csv\" using 4:(((column(1)+column(2)/1000000000)>$(plot_end_val)) ? ((v9=v9+1) <= 1 ? column(6) : NaN) : NaN) with points linecolor rgb \"dark-blue\" pointtype 2 title 'drone 9', \
						\"State10.csv\" using 4:(((column(1)+column(2)/1000000000)>$(plot_end_val)) ? ((v10=v10+1) <= 1 ? column(6) : NaN) : NaN) with points linecolor rgb \"dark-blue\" pointtype 2 title 'drone 10', \
						\"State11.csv\" using 4:(((column(1)+column(2)/1000000000)>$(plot_end_val)) ? ((v11=v11+1) <= 1 ? column(6) : NaN) : NaN) with points linecolor rgb \"dark-blue\" pointtype 2 title 'drone 11', \
						\"State12.csv\" using 4:(((column(1)+column(2)/1000000000)>$(plot_end_val)) ? ((v12=v12+1) <= 1 ? column(6) : NaN) : NaN) with points linecolor rgb \"dark-blue\" pointtype 2 title 'drone 13', \
						\"State13.csv\" using 4:(((column(1)+column(2)/1000000000)>$(plot_end_val)) ? ((v13=v13+1) <= 1 ? column(6) : NaN) : NaN) with points linecolor rgb \"dark-blue\" pointtype 2 title 'drone 13', \
						\"State14.csv\" using 4:(((column(1)+column(2)/1000000000)>$(plot_end_val)) ? ((v14=v14+1) <= 1 ? column(6) : NaN) : NaN) with points linecolor rgb \"dark-blue\" pointtype 2 title 'drone 14', \
						\"State15.csv\" using 4:(((column(1)+column(2)/1000000000)>$(plot_end_val)) ? ((v15=v15+1) <= 1 ? column(6) : NaN) : NaN) with points linecolor rgb \"dark-blue\" pointtype 2 title 'drone 15', \
						\"State16.csv\" using 4:(((column(1)+column(2)/1000000000)>$(plot_end_val)) ? ((v16=v16+1) <= 1 ? column(6) : NaN) : NaN) with points linecolor rgb \"dark-blue\" pointtype 2 title 'drone 16', \
						\"State0.csv\" using 7:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(9) : NaN) : NaN) with lines linecolor rgb \"light-red\" linewidth 1 title 'TARGET'" \
  	 | gnuplot


TraceYZ.pdf: State0.csv Makefile crazyflie2_*.yaml
	echo " \
  		set datafile separator ',' \n\
  		 \n\
		set term pdf font \"Times,15\" size 10cm, 7cm \n\
		set grid \n\
		v0 = 0 \n v1 = 0 \n v2 = 0 \n v3 = 0 \n v4 = 0 \n\
		v5 = 0 \n v6 = 0 \n v7 = 0 \n v8 = 0 \n v9 = 0 \n\
		v10 = 0 \n v11 = 0 \n v12 = 0 \n v13 = 0 \n v14 = 0 \n\
		v15 = 0 \n v16 = 0 \n v17 = 0 \n v18 = 0 \n v19 = 0 \n\
		set key off \n\
        set xtics 2 \n\
        set ytics 2 \n\
  		set xlabel \"Position y [m]\" \n\
  		set ylabel \"Position z [m]\" \n\
  		unset xlabel \n\
  		unset ylabel \n\
		set xrange [-1.0:14.0] \n\
		set yrange [-0.5:8.0] \n\
  		set output \"$@\" \n\
  		plot  		    \"State0.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linecolor rgb \"web-blue\" linewidth 1 title 'drone 0', \
						\"State1.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linecolor rgb \"web-blue\" linewidth 1 title 'drone 1', \
						\"State2.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linecolor rgb \"web-blue\" linewidth 1 title 'drone 2', \
						\"State3.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linecolor rgb \"web-blue\" linewidth 1 title 'drone 3', \
						\"State4.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linecolor rgb \"web-blue\" linewidth 1 title 'drone 4', \
						\"State5.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linecolor rgb \"web-blue\" linewidth 1 title 'drone 5', \
						\"State6.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linecolor rgb \"web-blue\" linewidth 1 title 'drone 6', \
						\"State7.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linecolor rgb \"web-blue\" linewidth 1 title 'drone 7', \
						\"State8.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linecolor rgb \"web-blue\" linewidth 1 title 'drone 8', \
						\"State9.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linecolor rgb \"web-blue\" linewidth 1 title 'drone 9', \
						\"State10.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linecolor rgb \"web-blue\" linewidth 1 title 'drone 10', \
						\"State11.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linecolor rgb \"web-blue\" linewidth 1 title 'drone 11', \
						\"State12.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linecolor rgb \"web-blue\" linewidth 1 title 'drone 12', \
						\"State13.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linecolor rgb \"web-blue\" linewidth 1 title 'drone 13', \
						\"State14.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linecolor rgb \"web-blue\" linewidth 1 title 'drone 14', \
						\"State15.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linecolor rgb \"web-blue\" linewidth 1 title 'drone 15', \
						\"State16.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linecolor rgb \"web-blue\" linewidth 1 title 'drone 16', \
						\"State0.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_end_val)) ? ((v0=v0+1) <= 1 ? column(6) : NaN) : NaN) with points linecolor rgb \"dark-blue\" pointtype 2 title 'drone 0', \
						\"State1.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_end_val)) ? ((v1=v1+1) <= 1 ? column(6) : NaN) : NaN) with points linecolor rgb \"dark-blue\" pointtype 2 title 'drone 1', \
						\"State2.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_end_val)) ? ((v2=v2+1) <= 1 ? column(6) : NaN) : NaN) with points linecolor rgb \"dark-blue\" pointtype 2 title 'drone 2', \
						\"State3.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_end_val)) ? ((v3=v3+1) <= 1 ? column(6) : NaN) : NaN) with points linecolor rgb \"dark-blue\" pointtype 2 title 'drone 3', \
						\"State4.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_end_val)) ? ((v4=v4+1) <= 1 ? column(6) : NaN) : NaN) with points linecolor rgb \"dark-blue\" pointtype 2 title 'drone 4', \
						\"State5.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_end_val)) ? ((v5=v5+1) <= 1 ? column(6) : NaN) : NaN) with points linecolor rgb \"dark-blue\" pointtype 2 title 'drone 5', \
						\"State6.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_end_val)) ? ((v6=v6+1) <= 1 ? column(6) : NaN) : NaN) with points linecolor rgb \"dark-blue\" pointtype 2 title 'drone 6', \
						\"State7.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_end_val)) ? ((v7=v7+1) <= 1 ? column(6) : NaN) : NaN) with points linecolor rgb \"dark-blue\" pointtype 2 title 'drone 7', \
						\"State8.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_end_val)) ? ((v8=v8+1) <= 1 ? column(6) : NaN) : NaN) with points linecolor rgb \"dark-blue\" pointtype 2 title 'drone 8', \
						\"State9.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_end_val)) ? ((v9=v9+1) <= 1 ? column(6) : NaN) : NaN) with points linecolor rgb \"dark-blue\" pointtype 2 title 'drone 9', \
						\"State10.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_end_val)) ? ((v10=v10+1) <= 1 ? column(6) : NaN) : NaN) with points linecolor rgb \"dark-blue\" pointtype 2 title 'drone 10', \
						\"State11.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_end_val)) ? ((v11=v11+1) <= 1 ? column(6) : NaN) : NaN) with points linecolor rgb \"dark-blue\" pointtype 2 title 'drone 11', \
						\"State13.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_end_val)) ? ((v13=v13+1) <= 1 ? column(6) : NaN) : NaN) with points linecolor rgb \"dark-blue\" pointtype 2 title 'drone 13', \
						\"State12.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_end_val)) ? ((v12=v12+1) <= 1 ? column(6) : NaN) : NaN) with points linecolor rgb \"dark-blue\" pointtype 2 title 'drone 12', \
						\"State14.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_end_val)) ? ((v14=v14+1) <= 1 ? column(6) : NaN) : NaN) with points linecolor rgb \"dark-blue\" pointtype 2 title 'drone 14', \
						\"State15.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_end_val)) ? ((v15=v15+1) <= 1 ? column(6) : NaN) : NaN) with points linecolor rgb \"dark-blue\" pointtype 2 title 'drone 15', \
						\"State16.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_end_val)) ? ((v16=v16+1) <= 1 ? column(6) : NaN) : NaN) with points linecolor rgb \"dark-blue\" pointtype 2 title 'drone 16', \
						\"State0.csv\" using 8:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(9) : NaN) : NaN) with lines linecolor rgb \"light-red\" linewidth 1 title 'TARGET'" \
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
	rm -f *.pdf

clean-all: clean rm-csv

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
