allDistanceCsv := $(shell ls DistSimDistance*.csv | grep -v Merged)
allDistanceTmp := $(subst DistSimDistance,Distance,$(allDistanceCsv))
allDistancePng := $(subst csv,png,$(allDistanceTmp))

allDistanceGTCsv := $(shell ls DistSimDistance*.csv | grep -v Merged)
allDistanceGTTmp := $(subst DistSimDistance,DistanceGT,$(allDistanceGTCsv))
allDistanceGTPng := $(subst csv,png,$(allDistanceGTTmp))

allMetricsCsv := $(shell ls DistSimMetrics*.csv | grep -v Merged)
allMetricsTmp := $(subst DistSimMetrics,Metrics,$(allMetricsCsv))
allMetricsPng := $(subst csv,png,$(allMetricsTmp))

allMinDist := $(subst DistSimMetrics,MinDist,$(allMetricsCsv))
allMaxDist := $(subst DistSimMetrics,MaxDist,$(allMetricsCsv))
allBeaconDist := $(subst DistSimMetrics,BeaconDist,$(allMetricsCsv))

allCentroidCsv := $(shell ls DistSimCentroid*.csv | grep -v Merged)
allCentroidTmp := $(subst DistSimCentroid,Centroid,$(allCentroidCsv))
allCentroidPng := $(subst csv,png,$(allCentroidTmp))

allStateCsv := $(shell ls State*.csv | grep -v Merged)
allStatePng := $(subst csv,png,$(allStateCsv))

allCostCsv := $(shell ls Cost*.csv | grep -v Merged)
allCostPng := $(subst csv,png,$(allCostCsv))

allEnvCsv := $(shell ls Env*.csv | grep -v Merged)
allEnvPng := $(subst csv,png,$(allEnvCsv))

allNoisePng := $(subst State,Noise,$(allStatePng))

all-distance: $(allDistancePng)
all-distanceGT: $(allDistanceGTPng)
all-metrics: $(allMetricsPng)
all-centroid: $(allCentroidPng)
all-state: $(allStatePng)
all-cost: $(allCostPng)
all-noise: $(allNoisePng)
all-trace: TraceXY.png TraceXZ.png TraceYZ.png

all: all-distance all-distanceGT all-metrics all-centroid all-state all-cost all-trace

plot_start_val := 0
plot_end_val := 9999999999999999999

Distance%.png: Distance%.csv Env%.csv DistSimDistance%.csv Makefile
	echo " \
		set datafile separator ',' \n\
		 \n\
		set term pngcairo size 1820,980 \n\
		set grid \n\
		set xlabel \"Time [s]\" \n\
		set ylabel \"Distance [m]\" \n\
		set y2label \"Count\" \n\
		set yrange [0:4] \n\
		set y2range [0:20] \n\
		set y2tics 0,1,20 \n\
		set xtics nomirror \n\
		set y2tics nomirror \n\
		set output \"$@\" \n\
		set palette rgb -22,-22,2; \n\
		set cbrange [-46:90] \n\
		unset colorbox \n\
		plot     \"$(word 2,$^)\" using (column(1)+column(2)/1000000000):(20):(column(4)/2) with boxes title '' linecolor palette axes x1y2, \
				   	 \"$<\" using (column(1)+column(2)/1000000000):(column(3)/4) with lines title '> enable/established/4' axes x1y2 lc rgb 'gold', \
				   	 \"$(word 2,$^)\" using (column(1)+column(2)/1000000000):(column(4)) with lines title '> explore/exploit' axes x1y2 lc 3, \
				   	 \"$(word 2,$^)\" using (column(1)+column(2)/1000000000):(column(9)/500+0.05) with lines title '< expl. cnt/500' lc 4, \
				   	 \"$<\" using (column(1)+column(2)/1000000000):4 with lines title 'dist to 0', \
				   	 \"$<\" using (column(1)+column(2)/1000000000):6 with lines title 'dist to 1', \
				   	 \"$<\" using (column(1)+column(2)/1000000000):8 with lines title 'dist to 2', \
				   	 \"$<\" using (column(1)+column(2)/1000000000):10 with lines title 'dist to 3', \
				   	 \"$<\" using (column(1)+column(2)/1000000000):12 with lines title 'dist to 4', \
				   	 \"$<\" using (column(1)+column(2)/1000000000):14 with lines title 'dist to 5', \
				   	 \"$<\" using (column(1)+column(2)/1000000000):16 with lines title 'dist to 6', \
				   	 \"$<\" using (column(1)+column(2)/1000000000):18 with lines title 'dist to 7', \
				   	 \"$<\" using (column(1)+column(2)/1000000000):20 with lines title 'dist to 8', \
				   	 \"$<\" using (column(1)+column(2)/1000000000):5 with lines title 'filtered dist to 0', \
				   	 \"$<\" using (column(1)+column(2)/1000000000):7 with lines title 'filtered dist to 1', \
				   	 \"$<\" using (column(1)+column(2)/1000000000):9 with lines title 'filtered dist to 2', \
				   	 \"$<\" using (column(1)+column(2)/1000000000):11 with lines title 'filtered dist to 3', \
				   	 \"$<\" using (column(1)+column(2)/1000000000):13 with lines title 'filtered dist to 4', \
				   	 \"$<\" using (column(1)+column(2)/1000000000):15 with lines title 'filtered dist to 5', \
				   	 \"$<\" using (column(1)+column(2)/1000000000):17 with lines title 'filtered dist to 6', \
				   	 \"$<\" using (column(1)+column(2)/1000000000):19 with lines title 'filtered dist to 7', \
				   	 \"$<\" using (column(1)+column(2)/1000000000):21 with lines title 'filtered dist to 8', \
				   	 \"$(word 3,$^)\" using (column(1)+column(2)/1000000000):4 with lines title 'GT dist to 0', \
				   	 \"$(word 3,$^)\" using (column(1)+column(2)/1000000000):5 with lines title 'GT dist to 1', \
				   	 \"$(word 3,$^)\" using (column(1)+column(2)/1000000000):6 with lines title 'GT dist to 2', \
				   	 \"$(word 3,$^)\" using (column(1)+column(2)/1000000000):7 with lines title 'GT dist to 3', \
				   	 \"$(word 3,$^)\" using (column(1)+column(2)/1000000000):8 with lines title 'GT dist to 4', \
				   	 \"$(word 3,$^)\" using (column(1)+column(2)/1000000000):9 with lines title 'GT dist to 5', \
				   	 \"$(word 3,$^)\" using (column(1)+column(2)/1000000000):10 with lines title 'GT dist to 6', \
				   	 \"$(word 3,$^)\" using (column(1)+column(2)/1000000000):11 with lines title 'GT dist to 7', \
				   	 \"$(word 3,$^)\" using (column(1)+column(2)/1000000000):12 with lines title 'GT dist to 8', \
				   	 0.28 with lines linewidth 2 lt rgb 'light-red' dashtype 2 title 'dist_{thr} (28cm)'" \
	 | gnuplot

DistanceGT%.png: DistSimDistance%.csv Env%.csv Makefile
	echo " \
		set datafile separator ',' \n\
		 \n\
		set term pngcairo size 1820,980 \n\
		set grid \n\
		set xlabel \"Time [s]\" \n\
		set ylabel \"Distance [m]\" \n\
		set y2label \"Count\" \n\
		set yrange [0:4] \n\
		set y2range [0:20] \n\
		set y2tics 0,1,20 \n\
		set xtics nomirror \n\
		set y2tics nomirror \n\
		set output \"$@\" \n\
		set palette rgb -22,-22,2; \n\
		set cbrange [-46:90] \n\
		unset colorbox \n\
		plot     \"$(word 2,$^)\" using (column(1)+column(2)/1000000000):(20):(column(4)/2) with boxes title '' linecolor palette axes x1y2, \
				   	 \"$<\" using (column(1)+column(2)/1000000000):(column(3)/4) with lines title '> enable/established/4' axes x1y2 lc rgb 'gold', \
				   	 \"$(word 2,$^)\" using (column(1)+column(2)/1000000000):(column(4)) with lines title '> explore/exploit' axes x1y2 lc 3, \
				   	 \"$(word 2,$^)\" using (column(1)+column(2)/1000000000):(column(9)/500+0.05) with lines title '< expl. cnt/500' lc 4, \
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

Metrics%.png: DistSimMetrics%.csv Env%.csv AllMinDist.csv AllMaxDist.csv AllBeaconDist.csv Makefile
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
		stats \"$<\" using ((column(3)>64) ? column(9) : NaN) \n\
		print sprintf(\"metric: MEAN of dist max %f\", STATS_mean) \n\
		print sprintf(\"metric: MEDIAN of dist max %f\", STATS_median) \n\
		print sprintf(\"metric: MIN of dist max %f\", STATS_min) \n\
		print sprintf(\"metric: MAX of dist max %f\", STATS_max) \n\
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
		set yrange [0:15] \n\
		set y2range [0:20] \n\
		set y2tics 0,1,20 \n\
		set xtics nomirror \n\
		set y2tics nomirror \n\
		set output \"$@\" \n\
		set palette rgb -22,-22,2; \n\
		set cbrange [-46:90] \n\
		unset colorbox \n\
		plot     \"$(word 2,$^)\" using (column(1)+column(2)/1000000000):(20):(column(4)/2) with boxes title '' linecolor palette axes x1y2, \
				 \"$<\" using (column(1)+column(2)/1000000000):(column(3)/4) with lines title '> enable/established/4' axes x1y2 lc rgb 'gold', \
				 \"$(word 2,$^)\" using (column(1)+column(2)/1000000000):(column(4)) with lines title '> explore/exploit' axes x1y2 lc 3, \
				 \"$(word 2,$^)\" using (column(1)+column(2)/1000000000):(column(9)/250+0.05) with lines title '< expl. cnt/250' lc 4, \
				 \"$<\" using (column(1)+column(2)/1000000000):4 with lines title 'drone count' axes x1y2 lc 4 dt 3, \
				 \"$<\" using (column(1)+column(2)/1000000000):5 with lines title 'beacon count' axes x1y2, \
				 \"$<\" using (column(1)+column(2)/1000000000):6 with lines title 'dist to drone min' lc 1 lw 2, \
				 \"$<\" using (column(1)+column(2)/1000000000):7 with lines title 'swarm radius' lc 2 lw 2, \
				 \"$<\" using (column(1)+column(2)/1000000000):8 with lines title 'dist to beacon0' lc 7, \
				 \"$<\" using (column(1)+column(2)/1000000000):9 with lines title 'dist to drone max' lc 6 lw 2, \
				 \"$(word 2,$^)\" using (column(1)+column(2)/1000000000):(column(5)+6) with lines title 'move_x' lc 9, \
				 \"$(word 2,$^)\" using (column(1)+column(2)/1000000000):(column(6)+7) with lines title 'move_y' lc 10, \
				 \"$(word 2,$^)\" using (column(1)+column(2)/1000000000):(column(7)+8) with lines title 'move_z' lc 11, \
				 \"AllMinDist.csv\" using (column(1)+column(2)/1000000000):3 with lines lc 4 lw 2 title 'min interagent dist', \
				 \"AllMaxDist.csv\" using (column(1)+column(2)/1000000000):3 with lines lc 6 lw 2 title 'max interagent dist', \
				 \"AllBeaconDist.csv\" using (column(1)+column(2)/1000000000):3 with lines lc 7 lw 2 title 'avg beacon dist', \
		   	     6 with lines linewidth 1 lc 9 dashtype 3 title '', \
		   	     7 with lines linewidth 1 lc 10 dashtype 3 title '', \
		   	     8 with lines linewidth 1 lc 11 dashtype 3 title '', \
		   	     0.28 with lines linewidth 2 lt rgb 'light-red' dashtype 2 title 'dist_{thr} (28cm)'" \
	 | gnuplot

State%.png: State%.csv Env%.csv Makefile
	echo " \
 		set datafile separator ',' \n\
	  \n\
	    stats \"$<\" using ((column(3)>64) ? column(4) : NaN) \n\
		print sprintf(\"metric: MEAN of pos x %f\", STATS_mean) \n\
		print sprintf(\"metric: MEDIAN of pos x %f\", STATS_median) \n\
		print sprintf(\"metric: MIN of pos x %f\", STATS_min) \n\
		print sprintf(\"metric: MAX of pos x %f\", STATS_max) \n\
	    stats \"$<\" using ((column(3)>64) ? column(5) : NaN) \n\
		print sprintf(\"metric: MEAN of pos y %f\", STATS_mean) \n\
		print sprintf(\"metric: MEDIAN of pos y %f\", STATS_median) \n\
		print sprintf(\"metric: MIN of pos y %f\", STATS_min) \n\
		print sprintf(\"metric: MAX of pos y %f\", STATS_max) \n\
	    stats \"$<\" using ((column(3)>64) ? column(6) : NaN) \n\
		print sprintf(\"metric: MEAN of pos z %f\", STATS_mean) \n\
		print sprintf(\"metric: MEDIAN of pos z %f\", STATS_median) \n\
		print sprintf(\"metric: MIN of pos z %f\", STATS_min) \n\
		print sprintf(\"metric: MAX of pos z %f\", STATS_max) \n\
			stats \"$(word 2,$^)\" using ((column(3)>64) ? column(10) : NaN) \n\
		print sprintf(\"metric: MEAN of cnt neighbours %f\", STATS_mean) \n\
		print sprintf(\"metric: MEDIAN of cnt neighbours %f\", STATS_median) \n\
		print sprintf(\"metric: MIN of cnt neighbours %f\", STATS_min) \n\
		print sprintf(\"metric: MAX of cnt neighbours %f\", STATS_max) \n\
	  \n\
 		set term pngcairo size 1820,980 \n\
 		set grid \n\
 		set xlabel \"Time [s]\" \n\
 		set ylabel \"Position $< [m]\" \n\
		set y2label \"Count\" \n\
		set y2range [0:20] \n\
		set y2tics 0,1,20 \n\
		set xtics nomirror \n\
		set y2tics nomirror \n\
 		set output \"$@\" \n\
		set palette rgb -22,-22,2; \n\
		set cbrange [-46:90] \n\
		unset colorbox \n\
		plot    \"$(word 2,$^)\" using (column(1)+column(2)/1000000000):(20):(column(4)/2) with boxes title '' linecolor palette axes x1y2, \
				\"$<\" using (column(1)+column(2)/1000000000):(column(3)/4) with lines title '> enable/established/4' axes x1y2 lc rgb 'gold', \
				\"$(word 2,$^)\" using (column(1)+column(2)/1000000000):(column(4)) with lines title '> explore/exploit' axes x1y2 lc 3, \
				\"$(word 2,$^)\" using (column(1)+column(2)/1000000000):(column(9)/250+0.05) with lines title '< expl. cnt/250' lc 4, \
				\"$(word 2,$^)\" using (column(1)+column(2)/1000000000):(column(10)+0.05) with lines title '< neighbours cnt' lc 5, \
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

Centroid%.png: DistSimCentroid%.csv Env%.csv Makefile
	echo " \
 		set datafile separator ',' \n\
	  \n\
 		set term pngcairo size 1820,980 \n\
 		set grid \n\
 		set xlabel \"Time [s]\" \n\
 		set ylabel \"Position $< [m]\" \n\
		set y2label \"Count\" \n\
		set yrange [-1:10] \n\
		set y2range [0:20] \n\
		set y2tics 0,1,20 \n\
		set xtics nomirror \n\
		set y2tics nomirror \n\
 		set output \"$@\" \n\
		set palette rgb -22,-22,2; \n\
		set cbrange [-46:90] \n\
		unset colorbox \n\
		plot    \"$(word 2,$^)\" using (column(1)+column(2)/1000000000):(20):(column(4)/2) with boxes title '' linecolor palette axes x1y2, \
				\"$(word 2,$^)\" using (column(1)+column(2)/1000000000):(column(3)/4) with lines title '> enable/established/4' axes x1y2 lc rgb 'gold', \
				\"$(word 2,$^)\" using (column(1)+column(2)/1000000000):(column(4)) with lines title '> explore/exploit' axes x1y2 lc 3, \
				\"$(word 2,$^)\" using (column(1)+column(2)/1000000000):(column(9)/250+0.05) with lines title '< expl. cnt/250' lc 4, \
				\"$<\" using (column(1)+column(2)/1000000000):4 with lines title 'centroid_x' lc 1, \
				\"$<\" using (column(1)+column(2)/1000000000):5 with lines title 'centroid_y' lc 2, \
				\"$<\" using (column(1)+column(2)/1000000000):6 with lines title 'centroid_z' lc 3, \
				\"$<\" using (column(1)+column(2)/1000000000):7 with lines title 'beacon0_x' lc 4, \
				\"$<\" using (column(1)+column(2)/1000000000):8 with lines title 'beacon0_y' lc 5, \
				\"$<\" using (column(1)+column(2)/1000000000):9 with lines title 'beacon0_z' lc 6, \
				\"$<\" using (column(1)+column(2)/1000000000):10 with lines title 'centroid to beacon0' lc 7" \
 	 | gnuplot

Cost%.png: Cost%.csv Env%.csv Makefile
	echo " \
 		set datafile separator ',' \n\
	  \n\
 		set term pngcairo size 1820,980 \n\
 		set grid \n\
 		set xlabel \"Time [s]\" \n\
 		set ylabel \"Position $< [m]\" \n\
		set y2label \"Count\" \n\
		set y2range [0:20] \n\
		set y2tics 0,1,20 \n\
		set xtics nomirror \n\
		set y2tics nomirror \n\
 		set output \"$@\" \n\
		set palette rgb -22,-22,2; \n\
		set cbrange [-46:90] \n\
		unset colorbox \n\
		plot    \"$(word 2,$^)\" using (column(1)+column(2)/1000000000):(20):(column(4)/2) with boxes title '' linecolor palette axes x1y2, \
				\"$<\" using (column(1)+column(2)/1000000000):(column(3)/4) with lines title '> enable/established/4' axes x1y2 lc rgb 'gold', \
				\"$(word 2,$^)\" using (column(1)+column(2)/1000000000):(column(4)) with lines title '> explore/exploit' axes x1y2 lc 3, \
				\"$(word 2,$^)\" using (column(1)+column(2)/1000000000):(column(9)/250+0.05) with lines title '< expl. cnt/250' lc 4, \
				\"$<\" using (column(1)+column(2)/1000000000):(column(4)/100) with lines title 'cost total /100', \
				\"$<\" using (column(1)+column(2)/1000000000):(column(5)/100) with lines title 'cost coh /100', \
				\"$<\" using (column(1)+column(2)/1000000000):(column(6)/100) with lines title 'cost sep /100', \
				\"$<\" using (column(1)+column(2)/1000000000):(column(7)/100) with lines title 'cost target /100', \
				\"$<\" using (column(1)+column(2)/1000000000):(column(8)/100) with lines title 'cost calm /100', \
				\"$<\" using (column(1)+column(2)/1000000000):(column(9)/100) with lines title 'cost height /100', \
			    \"$(word 2,$^)\" using (column(1)+column(2)/1000000000):(column(5)+6) with lines lc 9 title 'move_x', \
			    \"$(word 2,$^)\" using (column(1)+column(2)/1000000000):(column(6)+7) with lines lc 10 title 'move_y', \
			    \"$(word 2,$^)\" using (column(1)+column(2)/1000000000):(column(7)+8) with lines lc 11 title 'move_z', \
	       	    6 with lines linewidth 1 lc 9 dashtype 3 title '', \
	       	    7 with lines linewidth 1 lc 10 dashtype 3 title '', \
	       	    8 with lines linewidth 1 lc 11 dashtype 3 title ''" \
 	 | gnuplot

Target.png: DistSimTarget.csv Env00.csv Makefile
	echo " \
		set datafile separator ',' \n\
		 \n\
	    stats \"$<\" using (column(6)) \n\
		print sprintf(\"metric: MEAN of target vel %f\", STATS_mean) \n\
		print sprintf(\"metric: MEDIAN of target vel %f\", STATS_median) \n\
		print sprintf(\"metric: MIN of target vel %f\", STATS_min) \n\
		print sprintf(\"metric: MAX of target vel %f\", STATS_max) \n\
 		 \n\
		set term pngcairo size 1820,980 \n\
		set grid \n\
		set xlabel \"Time [s]\" \n\
		set ylabel \"Distance [m]\" \n\
		set y2label \"Count\" \n\
		set y2range [0:20] \n\
		set y2tics 0,1,20 \n\
		set xtics nomirror \n\
		set y2tics nomirror \n\
		set output \"$@\" \n\
		set palette rgb -22,-22,2; \n\
		set cbrange [-46:90] \n\
		unset colorbox \n\
		plot     \"$(word 2,$^)\" using (column(1)+column(2)/1000000000):(column(4)) with lines title '> explore/exploit' axes x1y2 lc 3, \
				 \"$(word 2,$^)\" using (column(1)+column(2)/1000000000):(column(9)/250+0.05) with lines title '< expl. cnt/250' lc 4, \
				 \"$<\" using (column(1)+column(2)/1000000000):6 with lines title '< target vel' lc 2 lw 2, \
		   	     0.28 with lines linewidth 2 lt rgb 'light-red' dashtype 2 title 'dist_{thr} (28cm)'" \
	 | gnuplot



TraceXY.png: State00.csv Makefile
	echo " \
  		set datafile separator ',' \n\
  		 \n\
		set term pngcairo size 1820,1820 \n\
		set grid \n\
		v0 = 0 \n v1 = 0 \n v2 = 0 \n v3 = 0 \n v4 = 0 \n\
		v5 = 0 \n v6 = 0 \n v7 = 0 \n v8 = 0 \n v9 = 0 \n\
		v10 = 0 \n v11 = 0 \n v12 = 0 \n v13 = 0 \n v14 = 0 \n\
		v15 = 0 \n v16 = 0 \n v17 = 0 \n v18 = 0 \n v19 = 0 \n\
        set xtics 2 \n\
        set ytics 2 \n\
  		set xlabel \"Position y [m]\" \n\
  		set ylabel \"Position x [m]\" \n\
		set xrange [-12.0:12.0] \n\
		set yrange [-12.0:12.0] \n\
  		set output \"$@\" \n\
  		plot  		    \"State00.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(4) : NaN) : NaN) with lines linewidth 1 title 'drone 0', \
						\"State01.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(4) : NaN) : NaN) with lines linewidth 1 title 'drone 1', \
						\"State02.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(4) : NaN) : NaN) with lines linewidth 1 title 'drone 2', \
						\"State03.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(4) : NaN) : NaN) with lines linewidth 1 title 'drone 3', \
						\"State04.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(4) : NaN) : NaN) with lines linewidth 1 title 'drone 4', \
						\"State05.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(4) : NaN) : NaN) with lines linewidth 1 title 'drone 5', \
						\"State06.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(4) : NaN) : NaN) with lines linewidth 1 title 'drone 6', \
						\"State07.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(4) : NaN) : NaN) with lines linewidth 1 title 'drone 7', \
						\"State08.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(4) : NaN) : NaN) with lines linewidth 1 title 'drone 8', \
						\"State09.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(4) : NaN) : NaN) with lines linewidth 1 title 'drone 9', \
						\"State10.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(4) : NaN) : NaN) with lines linewidth 1 title 'drone 10', \
						\"State11.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(4) : NaN) : NaN) with lines linewidth 1 title 'drone 11', \
						\"State13.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(4) : NaN) : NaN) with lines linewidth 1 title 'drone 13', \
						\"State12.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(4) : NaN) : NaN) with lines linewidth 1 title 'drone 12', \
						\"State14.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(4) : NaN) : NaN) with lines linewidth 1 title 'drone 14', \
						\"State15.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(4) : NaN) : NaN) with lines linewidth 1 title 'drone 15', \
						\"State16.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(4) : NaN) : NaN) with lines linewidth 1 title 'drone 16', \
						\"State00.csv\" using 8:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(7) : NaN) : NaN) with lines linecolor rgb \"light-red\" linewidth 1 title 'TARGET'" \
  	 | gnuplot



TraceXZ.png: State00.csv Makefile
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
		set xrange [-12.0:12.0] \n\
		set yrange [-0.5:9.0] \n\
  		set output \"$@\" \n\
	    plot  		    \"State00.csv\" using 4:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linewidth 1 title 'drone 0', \
						\"State01.csv\" using 4:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linewidth 1 title 'drone 1', \
						\"State02.csv\" using 4:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linewidth 1 title 'drone 2', \
						\"State03.csv\" using 4:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linewidth 1 title 'drone 3', \
						\"State04.csv\" using 4:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linewidth 1 title 'drone 4', \
						\"State05.csv\" using 4:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linewidth 1 title 'drone 5', \
						\"State06.csv\" using 4:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linewidth 1 title 'drone 6', \
						\"State07.csv\" using 4:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linewidth 1 title 'drone 7', \
						\"State08.csv\" using 4:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linewidth 1 title 'drone 8', \
						\"State09.csv\" using 4:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linewidth 1 title 'drone 9', \
						\"State10.csv\" using 4:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linewidth 1 title 'drone 10', \
						\"State11.csv\" using 4:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linewidth 1 title 'drone 11', \
						\"State12.csv\" using 4:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linewidth 1 title 'drone 12', \
						\"State13.csv\" using 4:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linewidth 1 title 'drone 13', \
						\"State14.csv\" using 4:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linewidth 1 title 'drone 14', \
						\"State15.csv\" using 4:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linewidth 1 title 'drone 15', \
						\"State16.csv\" using 4:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linewidth 1 title 'drone 16', \
						\"State00.csv\" using 7:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(9) : NaN) : NaN) with lines linecolor rgb \"light-red\" linewidth 1 title 'TARGET'" \
  	 | gnuplot


TraceYZ.png: State00.csv Makefile
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
		set xrange [-12.0:12.0] \n\
		set yrange [-0.5:9.0] \n\
  		set output \"$@\" \n\
  		plot  		    \"State00.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linewidth 1 title 'drone 0', \
						\"State01.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linewidth 1 title 'drone 1', \
						\"State02.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linewidth 1 title 'drone 2', \
						\"State03.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linewidth 1 title 'drone 3', \
						\"State04.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linewidth 1 title 'drone 4', \
						\"State05.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linewidth 1 title 'drone 5', \
						\"State06.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linewidth 1 title 'drone 6', \
						\"State07.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linewidth 1 title 'drone 7', \
						\"State08.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linewidth 1 title 'drone 8', \
						\"State09.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linewidth 1 title 'drone 9', \
						\"State10.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linewidth 1 title 'drone 10', \
						\"State11.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linewidth 1 title 'drone 11', \
						\"State12.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linewidth 1 title 'drone 12', \
						\"State13.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linewidth 1 title 'drone 13', \
						\"State14.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linewidth 1 title 'drone 14', \
						\"State15.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linewidth 1 title 'drone 15', \
						\"State16.csv\" using 5:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(6) : NaN) : NaN) with lines linewidth 1 title 'drone 16', \
						\"State00.csv\" using 8:(((column(1)+column(2)/1000000000)>$(plot_start_val)) ? (((column(1)+column(2)/1000000000)<$(plot_end_val)) ? column(9) : NaN) : NaN) with lines linecolor rgb \"light-red\" linewidth 1 title 'TARGET'" \
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

EnvMerged.csv: $(allEnvCsv)
	rm -rf $@
	cat $? > $@

EstDistance%.csv: Distance%.csv Makefile
	awk 'BEGIN { FS = "," } ; { if ($$1+$$2/1000000000 >= $(starttime) && $$1+$$2/1000000000 <= $(endtime)+1) { \
	min = 999; \
	max = 0; \
	if ($$5 < min && $$5 > 0) min = $$5; \
	if ($$7 < min && $$7 > 0) min = $$7; \
	if ($$9 < min && $$9 > 0) min = $$9; \
	if ($$11 < min && $$11 > 0) min = $$11; \
	if ($$13 < min && $$13 > 0) min = $$13; \
	if ($$5 > max && $$5 > 0) max = $$5; \
	if ($$7 > max && $$7 > 0) max = $$7; \
	if ($$9 > max && $$9 > 0) max = $$9; \
	if ($$11 > max && $$11 > 0) max = $$11; \
	if ($$13 > max && $$13 > 0) max = $$13; \
	print $$0 min FS max \
	 } }' $< > $@

MinDist%.csv: DistSimMetrics%.csv Makefile
	awk 'BEGIN { FS = "," } ; { print $$1 FS $$2 FS $$6 }' $< > $@

MaxDist%.csv: DistSimMetrics%.csv Makefile
	awk 'BEGIN { FS = "," } ; { print $$1 FS $$2 FS $$9 }' $< > $@

BeaconDist%.csv: DistSimMetrics%.csv Makefile
	awk 'BEGIN { FS = "," } ; { print $$1 FS $$2 FS $$8 }' $< > $@

AllMinDist.csv: $(allMinDist) Makefile
	paste -d "," $(allMinDist) | \
	awk 'BEGIN { FS = "," } ; { \
	min = 999; \
	if ($$3 < min && $$3 > 0) min = $$3; \
	if ($$6 < min && $$6 > 0) min = $$6; \
	if ($$9 < min && $$9 > 0) min = $$9; \
	if ($$12 < min && $$12 > 0) min = $$12; \
	if ($$15 < min && $$15 > 0) min = $$15; \
	if ($$18 < min && $$18 > 0) min = $$18; \
	if ($$21 < min && $$21 > 0) min = $$21; \
	if ($$24 < min && $$24 > 0) min = $$24; \
	if ($$27 < min && $$27 > 0) min = $$27; \
	if ($$30 < min && $$30 > 0) min = $$30; \
	if ($$33 < min && $$33 > 0) min = $$33; \
	if ($$36 < min && $$36 > 0) min = $$36; \
	if ($$39 < min && $$39 > 0) min = $$39; \
	if ($$42 < min && $$42 > 0) min = $$42; \
	if ($$45 < min && $$45 > 0) min = $$45; \
	print $$1 FS $$2 FS min \
	 }' > $@

AllMaxDist.csv: $(allMaxDist) Makefile
	paste -d "," $(allMaxDist) | \
	awk 'BEGIN { FS = "," } ; { \
	max = 0; \
	if ($$3 > max && $$3 > 0) max = $$3; \
	if ($$6 > max && $$6 > 0) max = $$6; \
	if ($$9 > max && $$9 > 0) max = $$9; \
	if ($$12 > max && $$12 > 0) max = $$12; \
	if ($$15 > max && $$15 > 0) max = $$15; \
	if ($$18 > max && $$18 > 0) max = $$18; \
	if ($$21 > max && $$21 > 0) max = $$21; \
	if ($$24 > max && $$24 > 0) max = $$24; \
	if ($$27 > max && $$27 > 0) max = $$27; \
	if ($$30 > max && $$30 > 0) max = $$30; \
	if ($$33 > max && $$33 > 0) max = $$33; \
	if ($$36 > max && $$36 > 0) max = $$36; \
	if ($$39 > max && $$39 > 0) max = $$39; \
	if ($$42 > max && $$42 > 0) max = $$42; \
	if ($$45 > max && $$45 > 0) max = $$45; \
	print $$1 FS $$2 FS max \
	 }' > $@

AllBeaconDist.csv: $(allBeaconDist) Makefile
	paste -d "," $(allBeaconDist) | \
	awk 'BEGIN { FS = "," } ; { \
	avgsum = 0; avgn = 0; \
	if ($$3 > 0) { avgsum += $$3; avgn ++; } \
	if ($$6 > 0) { avgsum += $$6; avgn ++; } \
	if ($$9 > 0) { avgsum += $$9; avgn ++; } \
	if ($$12 > 0) { avgsum += $$12; avgn ++; } \
	if ($$15 > 0) { avgsum += $$15; avgn ++; } \
	if ($$18 > 0) { avgsum += $$18; avgn ++; } \
	if ($$21 > 0) { avgsum += $$21; avgn ++; } \
	if ($$24 > 0) { avgsum += $$24; avgn ++; } \
	if ($$27 > 0) { avgsum += $$27; avgn ++; } \
	if ($$30 > 0) { avgsum += $$30; avgn ++; } \
	if ($$33 > 0) { avgsum += $$33; avgn ++; } \
	if ($$36 > 0) { avgsum += $$36; avgn ++; } \
	if ($$39 > 0) { avgsum += $$39; avgn ++; } \
	if ($$42 > 0) { avgsum += $$42; avgn ++; } \
	if ($$45 > 0) { avgsum += $$45; avgn ++; } \
	if (avgn == 0) { avgsum = 0; avgn = 1; } \
	print $$1 FS $$2 FS avgsum/avgn FS avgsum FS avgn \
	 }' > $@


all-step: \
DronePosition0.png \
DronePositionTrajectory0.png \
DroneAttitude0.png \
XeYe0.png \
DeltaCommands0.png \
OmegaCommand0.png \
RCommand0.png \
PQCommands0.png \
AttiudeCommands0.png \
PropellersVelocity0.png \
XYcontroller0.png \
Attiudecontroller0.png

DronePosition%.png: DronePosition%.csv Makefile
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
		plot     \"$<\" using (column(1)+column(2)/1000000000):3 with lines title 'X-Pos', \
				 \"$<\" using (column(1)+column(2)/1000000000):4 with lines title 'Y-Pos', \
				 \"$<\" using (column(1)+column(2)/1000000000):5 with lines title 'Z-Pos', \
				 \"$<\" using (column(1)+column(2)/1000000000):6 with lines title 'X-Velocity' axis x2y2, \
				 \"$<\" using (column(1)+column(2)/1000000000):7 with lines title 'Y-Velocity' axis x2y2, \
				 \"$<\" using (column(1)+column(2)/1000000000):8 with lines title 'Z-Velocity' axis x2y2" \
	 | gnuplot
#	 set xrange [20:25] \n\
#	 set x2range [20:25] \n\
#	 set yrange [7.8:8.6] \n\

DronePositionTrajectory%.png: DronePosition%.csv Trajectory%.csv Makefile
	echo " \
		set datafile separator ',' \n\
		 \n\
		set term pngcairo size 1024,768 \n\
		set grid \n\
		set xlabel \"Time [s]\" \n\
		 \n\
		set output \"$@\" \n\
		set ylabel \"DronePosition (given by ground-truth odometry) and Trajectory (set-point commands)\" \n\
		plot     \"$(word 1,$^)\" using (column(1)+column(2)/1000000000):3 with lines title 'X-Pos', \
			     \"$(word 1,$^)\" using (column(1)+column(2)/1000000000):4 with lines title 'Y-Pos', \
			     \"$(word 1,$^)\" using (column(1)+column(2)/1000000000):5 with lines title 'Z-Pos', \
			     \"$(word 2,$^)\" using (column(1)+column(2)/1000000000):3 with lines title 'X-command', \
		     	 \"$(word 2,$^)\" using (column(1)+column(2)/1000000000):4 with lines title 'Y-command', \
		     	 \"$(word 2,$^)\" using (column(1)+column(2)/1000000000):5 with lines title 'Z-command'" \
	 | gnuplot

DroneAttitude%.png: DroneAttitude%.csv Makefile
	echo " \
		set datafile separator ',' \n\
		 \n\
		set term pngcairo size 1024,768 \n\
		set grid \n\
		set xlabel \"Time [s]\" \n\
		 \n\
		set output \"$@\" \n\
		set ylabel \"Drone attitude (given by ground-truth odometry (?))\" \n\
		plot     \"$<\" using (column(1)+column(2)/1000000000):3 with lines title 'roll', \
			     \"$<\" using (column(1)+column(2)/1000000000):4 with lines title 'pitch', \
			     \"$<\" using (column(1)+column(2)/1000000000):5 with lines title 'yaw'" \
	 | gnuplot

XeYe%.png: XeYe%.csv Makefile
	echo " \
		set datafile separator ',' \n\
		 \n\
		set term pngcairo size 1024,768 \n\
		set grid \n\
		set xlabel \"Time [s]\" \n\
		 \n\
		set output \"$@\" \n\
		set ylabel \"Drone position and velocity errors (of XYController)\" \n\
		plot     \"$<\" using (column(1)+column(2)/1000000000):3 with lines title 'X-error', \
			     \"$<\" using (column(1)+column(2)/1000000000):4 with lines title 'Y-error', \
			     \"$<\" using (column(1)+column(2)/1000000000):5 with lines title 'X-velocity-error', \
			     \"$<\" using (column(1)+column(2)/1000000000):6 with lines title 'Y-velocity-error'" \
	 | gnuplot

DeltaCommands%.png: DeltaCommands%.csv Makefile
	echo " \
		set datafile separator ',' \n\
		 \n\
		set term pngcairo size 1024,768 \n\
		set grid \n\
		set xlabel \"Time [s]\" \n\
		 \n\
		set output \"$@\" \n\
		set ylabel \"Delta Commands (of RateController)\" \n\
		plot     \"$<\" using (column(1)+column(2)/1000000000):3 with lines title 'delta-phi', \
			     \"$<\" using (column(1)+column(2)/1000000000):4 with lines title 'delta-theta', \
			     \"$<\" using (column(1)+column(2)/1000000000):5 with lines title 'delta-psi'" \
	 | gnuplot

OmegaCommand%.png: OmegaCommand%.csv Makefile
	echo " \
		set datafile separator ',' \n\
		 \n\
		set term pngcairo size 1024,768 \n\
		set grid \n\
		set xlabel \"Time [s]\" \n\
		 \n\
		set output \"$@\" \n\
		set ylabel \"Omega Command (of HoveringController)\" \n\
		plot     \"$<\" using (column(1)+column(2)/1000000000):3 with lines title 'omega-command'" \
	 | gnuplot

RCommand%.png: RCommand%.csv Makefile
	echo " \
		set datafile separator ',' \n\
		 \n\
		set term pngcairo size 1024,768 \n\
		set grid \n\
		set xlabel \"Time [s]\" \n\
		 \n\
		set output \"$@\" \n\
		set ylabel \"R Command (of YawMpcController)\" \n\
		plot     \"$<\" using (column(1)+column(2)/1000000000):3 with lines title 'r-command'" \
	 | gnuplot

PQCommands%.png: PQCommands%.csv Makefile
	echo " \
		set datafile separator ',' \n\
		 \n\
		set term pngcairo size 1024,768 \n\
		set grid \n\
		set xlabel \"Time [s]\" \n\
		 \n\
		set output \"$@\" \n\
		set ylabel \"PQ Commands (of AttitudeController)\" \n\
		plot    \"$<\" using (column(1)+column(2)/1000000000):3 with lines title 'p-command', \
	 		    \"$<\" using (column(1)+column(2)/1000000000):4 with lines title 'q-command'" \
	 | gnuplot

AttiudeCommands%.png: PQCommands%.csv DroneAttitude%.csv DronePosition%.csv Makefile
	echo " \
		set datafile separator ',' \n\
		 \n\
		set term pngcairo size 1024,768 \n\
		set grid \n\
		set xlabel \"Time [s]\" \n\
		 \n\
		set output \"$@\" \n\
		set ylabel \"Attitude and Commands\" \n\
		plot     \"$(word 1,$^)\" using (column(1)+column(2)/1000000000):3 with lines title 'phi-command', \
			     \"$(word 1,$^)\" using (column(1)+column(2)/1000000000):4 with lines title 'theta-command', \
			     \"$(word 2,$^)\" using (column(1)+column(2)/1000000000):3 with lines title 'pitch', \
			     \"$(word 2,$^)\" using (column(1)+column(2)/1000000000):4 with lines title 'roll', \
			     \"$(word 3,$^)\" using (column(1)+column(2)/1000000000):3 with lines title 'X-Pos', \
			     \"$(word 3,$^)\" using (column(1)+column(2)/1000000000):4 with lines title 'Y-Pos'" \
	 | gnuplot

PropellersVelocity%.png: PropellersVelocity%.csv Makefile
	echo " \
		set datafile separator ',' \n\
		 \n\
		set term pngcairo size 1024,768 \n\
		set grid \n\
		set xlabel \"Time [s]\" \n\
		 \n\
		set output \"$@\" \n\
		set ylabel \"Propellers velocity\" \n\
		plot     \"$<\" using (column(1)+column(2)/1000000000):3 with lines title 'A', \
			     \"$<\" using (column(1)+column(2)/1000000000):4 with lines title 'B', \
			     \"$<\" using (column(1)+column(2)/1000000000):5 with lines title 'C', \
			     \"$<\" using (column(1)+column(2)/1000000000):6 with lines title 'D'" \
	 | gnuplot

XYcontroller%.png: XeYe%.csv CommandAttitude%.csv Makefile
	echo " \
		set datafile separator ',' \n\
		 \n\
		set term pngcairo size 1024,768 \n\
		set grid \n\
		set xlabel \"Time [s]\" \n\
		 \n\
		set output \"$@\" \n\
		set ylabel \"XYcontroller\" \n\
		plot     \"$(word 1,$^)\" using (column(1)+column(2)/1000000000):3 with lines title 'error-x', \
			     \"$(word 1,$^)\" using (column(1)+column(2)/1000000000):4 with lines title 'error-y', \
			     \"$(word 1,$^)\" using (column(1)+column(2)/1000000000):5 with lines title 'error-Vx', \
			     \"$(word 1,$^)\" using (column(1)+column(2)/1000000000):6 with lines title 'error-Vy', \
			     \"$(word 2,$^)\" using (column(1)+column(2)/1000000000):3 with lines title 'theta-command', \
			     \"$(word 2,$^)\" using (column(1)+column(2)/1000000000):4 with lines title 'phi-command'" \
	 | gnuplot

Attiudecontroller%.png: CommandAttitude%.csv DroneAttitude%.csv PQCommands%.csv Makefile
	echo " \
		set datafile separator ',' \n\
		 \n\
		set term pngcairo size 1024,768 \n\
		set grid \n\
		set xlabel \"Time [s]\" \n\
		 \n\
		set output \"$@\" \n\
		set ylabel \"Attiudecontroller\" \n\
		plot     \"$(word 1,$^)\" using (column(1)+column(2)/1000000000):3 with lines title 'theta-command', \
		       	 \"$(word 1,$^)\" using (column(1)+column(2)/1000000000):4 with lines title 'phi-command', \
			     \"$(word 2,$^)\" using (column(1)+column(2)/1000000000):4 with lines title 'pitch (theta)', \
			     \"$(word 2,$^)\" using (column(1)+column(2)/1000000000):3 with lines title 'roll (phi)', \
		     	 \"$(word 3,$^)\" using (column(1)+column(2)/1000000000):4 with lines title 'q-command (theta-command)', \
			     \"$(word 3,$^)\" using (column(1)+column(2)/1000000000):3 with lines title 'p-command (phi-command)'" \
	 | gnuplot




rm-csv:
	rm -f *.csv

clean:
	rm -f *.png
	rm -f *.pdf
	rm -f *Merged.csv

clean-all: clean rm-csv
