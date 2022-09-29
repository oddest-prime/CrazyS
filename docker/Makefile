allDistanceCsv := $(shell ls DistSimDistance*.csv | grep -v Merged)
allDistanceTmp := $(subst DistSimDistance,Distance,$(allDistanceCsv))
allDistancePng := $(subst csv,png,$(allDistanceTmp))

allMetricsCsv := $(shell ls DistSimMetrics*.csv | grep -v Merged)
allMetricsTmp := $(subst DistSimMetrics,Metrics,$(allMetricsCsv))
allMetricsPng := $(subst csv,png,$(allMetricsTmp))

allStateCsv := $(shell ls State*.csv | grep -v Merged)
allStatePng := $(subst csv,png,$(allStateCsv))
allNoisePng := $(subst State,Noise,$(allStatePng))

all-distance: $(allDistancePng)
all-metrics: $(allMetricsPng)
all-state: $(allStatePng)
all-noise: $(allNoisePng)
all-trace: TraceXY.png TraceXZ.png TraceYZ.png

plot_start_val := 0
plot_end_val := 9999999999999999999

Distance%.png: DistSimDistance%.csv Env%.csv Makefile
	echo " \
		set datafile separator ',' \n\
		 \n\
		set term pngcairo size 1820,980 \n\
		set grid \n\
		set xlabel \"Time [s]\" \n\
		set ylabel \"Distance [m]\" \n\
		set y2label \"Count\" \n\
		set yrange [0:3] \n\
		set y2range [0:20] \n\
		set y2tics 0,1,20 \n\
		set xtics nomirror \n\
		set y2tics nomirror \n\
		set output \"$@\" \n\
		set palette rgb -22,-22,2; \n\
		set cbrange [-40:90] \n\
		unset colorbox \n\
		plot     \"$(word 2,$^)\" using (column(1)+column(2)/1000000000):(20):(column(4)/4) with boxes linecolor palette axes x1y2, \
				 \"$<\" using (column(1)+column(2)/1000000000):(column(3)/4) with lines title 'enable/established' axes x1y2 lc 4, \
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
				   	 \"$<\" using (column(1)+column(2)/1000000000):23 with lines title 'dist to 19', \
				   	 0.28 with lines linewidth 2 lt rgb 'light-red' dashtype 2 title 'dist_{thr} (28cm)'" \
	 | gnuplot

Metrics%.png: DistSimMetrics%.csv Env%.csv Makefile
	echo " \
		set datafile separator ',' \n\
		 \n\
		stats \"$<\" using ((column(3)>64) ? column(4) : NaN) \n\
		print sprintf(\"metric: MAX of drone count %f\", STATS_max) \n\
		stats \"$<\" using ((column(3)>64) ? column(5) : NaN) \n\
		print sprintf(\"metric: MAX of beacon count %f\", STATS_max) \n\
	    stats \"$<\" using ((column(3)>64) ? column(6) : NaN) \n\
		print sprintf(\"metric: MEAN of dist min %f\", STATS_mean) \n\
		print sprintf(\"metric: MEDIAN of dist min %f\", STATS_median) \n\
		print sprintf(\"metric: MIN of dist min %f\", STATS_min) \n\
		print sprintf(\"metric: MAX of dist min %f\", STATS_max) \n\
		stats \"$<\" using ((column(3)>64) ? column(7) : NaN) \n\
		print sprintf(\"metric: MEAN of swarm radius %f\", STATS_mean) \n\
		print sprintf(\"metric: MEDIAN of swarm radius %f\", STATS_median) \n\
		print sprintf(\"metric: MIN of swarm radius %f\", STATS_min) \n\
		print sprintf(\"metric: MAX of swarm radius %f\", STATS_max) \n\
		stats \"$<\" using ((column(3)>64) ? column(8) : NaN) \n\
		print sprintf(\"metric: MEAN of dist beacon %f\", STATS_mean) \n\
		print sprintf(\"metric: MEDIAN of dist beacon %f\", STATS_median) \n\
		print sprintf(\"metric: MIN of dist beacon %f\", STATS_min) \n\
		print sprintf(\"metric: MAX of dist beacon %f\", STATS_max) \n\
 		 \n\
		set term pngcairo size 1820,980 \n\
		set grid \n\
		set xlabel \"Time [s]\" \n\
		set ylabel \"Distance [m]\" \n\
		set y2label \"Count\" \n\
		set yrange [0:10] \n\
		set y2range [0:20] \n\
		set y2tics 0,1,20 \n\
		set xtics nomirror \n\
		set y2tics nomirror \n\
		set output \"$@\" \n\
		set palette rgb -22,-22,2; \n\
		set cbrange [-40:90] \n\
		unset colorbox \n\
		plot     \"$(word 2,$^)\" using (column(1)+column(2)/1000000000):(20):(column(4)/4) with boxes linecolor palette axes x1y2, \
				 \"$<\" using (column(1)+column(2)/1000000000):(column(3)/4) with lines title 'enable/established' axes x1y2 lc 4, \
				 \"$(word 2,$^)\" using (column(1)+column(2)/1000000000):(column(4)/4) with lines title 'explore/exploit' axes x1y2 lc 3, \
				 \"$<\" using (column(1)+column(2)/1000000000):4 with lines title 'drone count' axes x1y2 lc 7, \
				 \"$<\" using (column(1)+column(2)/1000000000):5 with lines title 'beacon count' axes x1y2, \
				 \"$<\" using (column(1)+column(2)/1000000000):6 with lines title 'dist to drone min' lc 1 lw 2, \
				 \"$<\" using (column(1)+column(2)/1000000000):7 with lines title 'swarm radius' lc 2 lw 2, \
				 \"$<\" using (column(1)+column(2)/1000000000):8 with lines title 'dist to beacon', \
				 \"$(word 2,$^)\" using (column(1)+column(2)/1000000000):(column(5)+6) with lines title 'move_x', \
				 \"$(word 2,$^)\" using (column(1)+column(2)/1000000000):(column(6)+7) with lines title 'move_y', \
				 \"$(word 2,$^)\" using (column(1)+column(2)/1000000000):(column(7)+8) with lines title 'move_z', \
		   	     6 with lines linewidth 1 lc 9 dashtype 3 title '', \
		   	     7 with lines linewidth 1 lc 10 dashtype 3 title '', \
		   	     8 with lines linewidth 1 lc 11 dashtype 3 title '', \
		   	     0.28 with lines linewidth 2 lt rgb 'light-red' dashtype 2 title 'dist_{thr} (28cm)'" \
	 | gnuplot

State%.png: State%.csv Env%.csv Makefile
	echo " \
 		set datafile separator ',' \n\
	  \n\
 		set term pngcairo size 1820,980 \n\
 		set grid \n\
 		set xlabel \"Time [s]\" \n\
 		set ylabel \"Position $< [m]\" \n\
		set y2label \"Count\" \n\
		set y2range [0:20] \n\
		set xtics nomirror \n\
		set y2tics nomirror \n\
 		set output \"$@\" \n\
		set palette rgb -22,-22,2; \n\
		set cbrange [-40:90] \n\
		unset colorbox \n\
		plot    \"$(word 2,$^)\" using (column(1)+column(2)/1000000000):(20):(column(4)/4) with boxes linecolor palette axes x1y2, \
				\"$<\" using (column(1)+column(2)/1000000000):(column(3)/4) with lines title 'enable/established' axes x1y2 lc 4, \
				\"$<\" using (column(1)+column(2)/1000000000):4 with lines title 'pos_x' lc 1, \
				\"$<\" using (column(1)+column(2)/1000000000):5 with lines title 'pos_y' lc 2, \
				\"$<\" using (column(1)+column(2)/1000000000):6 with lines title 'pos_z' lc 3, \
			    \"$(word 2,$^)\" using (column(1)+column(2)/1000000000):(column(5)+6) with lines lc 9 title 'move_x', \
			    \"$(word 2,$^)\" using (column(1)+column(2)/1000000000):(column(6)+7) with lines lc 10 title 'move_y', \
			    \"$(word 2,$^)\" using (column(1)+column(2)/1000000000):(column(7)+8) with lines lc 11 title 'move_z', \
	       	    6 with lines linewidth 1 lc 9 dashtype 3 title '', \
	       	    7 with lines linewidth 1 lc 10 dashtype 3 title '', \
	       	    8 with lines linewidth 1 lc 11 dashtype 3 title ''" \
 	 | gnuplot

TraceXY.png: State0.csv Makefile
	echo " \
  		set datafile separator ',' \n\
  		 \n\
		set term pngcairo size 1820,980 \n\
		set grid \n\
		v0 = 0 \n v1 = 0 \n v2 = 0 \n v3 = 0 \n v4 = 0 \n\
		v5 = 0 \n v6 = 0 \n v7 = 0 \n v8 = 0 \n v9 = 0 \n\
		v10 = 0 \n v11 = 0 \n v12 = 0 \n v13 = 0 \n v14 = 0 \n\
		v15 = 0 \n v16 = 0 \n v17 = 0 \n v18 = 0 \n v19 = 0 \n\
        set xtics 2 \n\
        set ytics 2 \n\
  		set xlabel \"Position y [m]\" \n\
  		set ylabel \"Position x [m]\" \n\
		set xrange [-7.0:7.0] \n\
		set yrange [-7.0:7.0] \n\
  		set output \"$@\" \n\
  		plot  		    \"State0.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(4) : NaN) : NaN) with lines linewidth 1 title 'drone 0', \
						\"State1.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(4) : NaN) : NaN) with lines linewidth 1 title 'drone 1', \
						\"State2.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(4) : NaN) : NaN) with lines linewidth 1 title 'drone 2', \
						\"State3.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(4) : NaN) : NaN) with lines linewidth 1 title 'drone 3', \
						\"State4.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(4) : NaN) : NaN) with lines linewidth 1 title 'drone 4', \
						\"State5.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(4) : NaN) : NaN) with lines linewidth 1 title 'drone 5', \
						\"State6.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(4) : NaN) : NaN) with lines linewidth 1 title 'drone 6', \
						\"State7.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(4) : NaN) : NaN) with lines linewidth 1 title 'drone 7', \
						\"State8.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(4) : NaN) : NaN) with lines linewidth 1 title 'drone 8', \
						\"State9.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(4) : NaN) : NaN) with lines linewidth 1 title 'drone 9', \
						\"State10.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(4) : NaN) : NaN) with lines linewidth 1 title 'drone 10', \
						\"State11.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(4) : NaN) : NaN) with lines linewidth 1 title 'drone 11', \
						\"State13.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(4) : NaN) : NaN) with lines linewidth 1 title 'drone 13', \
						\"State12.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(4) : NaN) : NaN) with lines linewidth 1 title 'drone 12', \
						\"State14.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(4) : NaN) : NaN) with lines linewidth 1 title 'drone 14', \
						\"State15.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(4) : NaN) : NaN) with lines linewidth 1 title 'drone 15', \
						\"State16.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(4) : NaN) : NaN) with lines linewidth 1 title 'drone 16', \
						\"State0.csv\" using 8:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(7) : NaN) : NaN) with lines linecolor rgb \"light-red\" linewidth 1 title 'TARGET'" \
  	 | gnuplot



TraceXZ.png: State0.csv Makefile
	echo " \
  		set datafile separator ',' \n\
  		 \n\
		set term pngcairo size 1820,980 \n\
		set grid \n\
		v0 = 0 \n v1 = 0 \n v2 = 0 \n v3 = 0 \n v4 = 0 \n\
		v5 = 0 \n v6 = 0 \n v7 = 0 \n v8 = 0 \n v9 = 0 \n\
		v10 = 0 \n v11 = 0 \n v12 = 0 \n v13 = 0 \n v14 = 0 \n\
		v15 = 0 \n v16 = 0 \n v17 = 0 \n v18 = 0 \n v19 = 0 \n\
        set xtics 2 \n\
        set ytics 2 \n\
  		set xlabel \"Position x [m]\" \n\
  		set ylabel \"Position z [m]\" \n\
		set xrange [-7.0:7.0] \n\
		set yrange [-0.5:8.0] \n\
  		set output \"$@\" \n\
	    plot  		    \"State0.csv\" using 4:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linewidth 1 title 'drone 0', \
						\"State1.csv\" using 4:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linewidth 1 title 'drone 1', \
						\"State2.csv\" using 4:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linewidth 1 title 'drone 2', \
						\"State3.csv\" using 4:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linewidth 1 title 'drone 3', \
						\"State4.csv\" using 4:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linewidth 1 title 'drone 4', \
						\"State5.csv\" using 4:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linewidth 1 title 'drone 5', \
						\"State6.csv\" using 4:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linewidth 1 title 'drone 6', \
						\"State7.csv\" using 4:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linewidth 1 title 'drone 7', \
						\"State8.csv\" using 4:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linewidth 1 title 'drone 8', \
						\"State9.csv\" using 4:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linewidth 1 title 'drone 9', \
						\"State10.csv\" using 4:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linewidth 1 title 'drone 10', \
						\"State11.csv\" using 4:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linewidth 1 title 'drone 11', \
						\"State12.csv\" using 4:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linewidth 1 title 'drone 12', \
						\"State13.csv\" using 4:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linewidth 1 title 'drone 13', \
						\"State14.csv\" using 4:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linewidth 1 title 'drone 14', \
						\"State15.csv\" using 4:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linewidth 1 title 'drone 15', \
						\"State16.csv\" using 4:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linewidth 1 title 'drone 16', \
						\"State0.csv\" using 7:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(9) : NaN) : NaN) with lines linecolor rgb \"light-red\" linewidth 1 title 'TARGET'" \
  	 | gnuplot


TraceYZ.png: State0.csv Makefile
	echo " \
  		set datafile separator ',' \n\
  		 \n\
		set term pngcairo size 1820,980 \n\
		set grid \n\
		v0 = 0 \n v1 = 0 \n v2 = 0 \n v3 = 0 \n v4 = 0 \n\
		v5 = 0 \n v6 = 0 \n v7 = 0 \n v8 = 0 \n v9 = 0 \n\
		v10 = 0 \n v11 = 0 \n v12 = 0 \n v13 = 0 \n v14 = 0 \n\
		v15 = 0 \n v16 = 0 \n v17 = 0 \n v18 = 0 \n v19 = 0 \n\
        set xtics 2 \n\
        set ytics 2 \n\
  		set xlabel \"Position y [m]\" \n\
  		set ylabel \"Position z [m]\" \n\
		set xrange [-7.0:7.0] \n\
		set yrange [-0.5:8.0] \n\
  		set output \"$@\" \n\
  		plot  		    \"State0.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linewidth 1 title 'drone 0', \
						\"State1.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linewidth 1 title 'drone 1', \
						\"State2.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linewidth 1 title 'drone 2', \
						\"State3.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linewidth 1 title 'drone 3', \
						\"State4.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linewidth 1 title 'drone 4', \
						\"State5.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linewidth 1 title 'drone 5', \
						\"State6.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linewidth 1 title 'drone 6', \
						\"State7.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linewidth 1 title 'drone 7', \
						\"State8.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linewidth 1 title 'drone 8', \
						\"State9.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linewidth 1 title 'drone 9', \
						\"State10.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linewidth 1 title 'drone 10', \
						\"State11.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linewidth 1 title 'drone 11', \
						\"State12.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linewidth 1 title 'drone 12', \
						\"State13.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linewidth 1 title 'drone 13', \
						\"State14.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linewidth 1 title 'drone 14', \
						\"State15.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linewidth 1 title 'drone 15', \
						\"State16.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linewidth 1 title 'drone 16', \
						\"State0.csv\" using 8:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(9) : NaN) : NaN) with lines linecolor rgb \"light-red\" linewidth 1 title 'TARGET'" \
  	 | gnuplot



CleanedMetrics%.csv: Metrics%.csv
	grep ",65," $< > $@
#	grep ",96," $< > $@
#	grep ",32," $< > $@

DistSimMetricsMerged.csv: $(allMetricsCsv)
	rm -rf $@
	cat $? > $@

StateMerged.csv: $(allStateCsv)
	rm -rf $@
	cat $? > $@

EnvMerged.csv:
	touch $@

rm-csv:
	rm -f *.csv

clean:
	rm -f *.png
	rm -f *.pdf
	rm -f *Merged.csv

clean-all: clean rm-csv
