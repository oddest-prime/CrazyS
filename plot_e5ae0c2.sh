#!/bin/bash

hash="e5ae0c2"
hash="06ae96d"
hash="4d06d98"
hash="0d4686a"
hash="05afa35"
hash="55946a5"
hash="2e4d40b"

rm -f data*.csv

for fn in `ls | grep ${hash}_ | grep -v video`
do
	ndrones=`echo $fn | cut -d "_" -f 5 | cut -d "m" -f 2`;
	ndrones2=`printf "%02d" $ndrones`
	mode=`echo $fn | grep -o -e "gradient" -e "gradenum" | head -n 1`;
	obstacles=`echo $fn | grep -q "obstacle2" && echo "3" || echo "0"`;
	echo $fn | grep -q "obstacle3" && obstacles="11";
	obstacles2=`printf "%02d" $obstacles`
	neighbourhood_distance=`grep "neighbourhood_distance:" $fn/simulation.info | cut -d " " -f 4 | cut -d "," -f 1`
	controller=`grep "controller:" $fn/simulation.info | cut -d " " -f 4`
	min_dist_min=`grep "MIN of dist min" $fn/simulation.info | cut -d " " -f 6`
	med_radius=`grep "MEDIAN of swarm radius" $fn/simulation.info | cut -d " " -f 6`
	max_radius=`grep "MAX of swarm radius" $fn/simulation.info | cut -d " " -f 6`
	min_obstacle=`grep "MIN of dist obstacle" $fn/simulation.info | cut -d " " -f 7`
	
	echo "$ndrones2,controller:$controller,mode:$mode,ndrones:$ndrones,obstacles:$obstacles,neighbourhood_distance:$neighbourhood_distance,min_dist_min:$min_dist_min,med_radius:$med_radius,max_radius:$max_radius,min_obstacle:$min_obstacle,fn:$fn" | tee -a data_all.csv
	#echo "$mode,$obstacles,$neighbourhood_distance,$ndrones2,$ndrones,$controller,$min_dist_min,$max_radius,$min_obstacle,$fn" | tee -a data_raw.csv
	#echo "$obstacles,$neighbourhood_distance,$ndrones2,$ndrones,$mode,$controller,$min_dist_min,$max_radius,$min_obstacle,$fn" | tee -a data_raw2.csv
	echo "$mode,$obstacles2,$neighbourhood_distance,$ndrones2,$ndrones,$controller,$min_dist_min,$max_radius,$min_obstacle,$fn,$ndrones dr. $obstacles obs." >> data_raw.csv
	echo "$obstacles2,$neighbourhood_distance,$ndrones2,$ndrones,$mode,$controller,$min_dist_min,$max_radius,$min_obstacle,$fn" >> data_raw2.csv
done;

echo "=========================="
sort data_raw.csv > data_sorted.csv
sort data_raw2.csv > data_sorted2.csv
echo "=========================="

#cat data_sorted.csv

echo " "
echo " "
echo " "
echo " "

#exit 0;

while read -r line
do
	continue;

	mode=`echo "$line" | cut -d "," -f 1`
	obstacles=`echo "$line" | cut -d "," -f 2`
	neighbourhood_distance=`echo "$line" | cut -d "," -f 3`
	ndrones2=`echo "$line" | cut -d "," -f 4`
	ndrones=`echo "$line" | cut -d "," -f 5`
	controller=`echo "$line" | cut -d "," -f 6`
	min_dist_min=`echo "$line" | cut -d "," -f 7`
	max_radius=`echo "$line" | cut -d "," -f 8`
	min_obstacle=`echo "$line" | cut -d "," -f 9`

	if [ "$neighbourhood_distance" = "999.0" ]
	then
		neighbourhood_distance="\infty"
	fi;
	
	if [ "$controller" = "1" ]
	then
		controller="A"
	fi;
	if [ "$controller" = "2" ]
	then
		controller="B"
	fi;

#	min_dist_min=`echo "$min_dist_min" | tr ".", ","`
	min_dist_min=`printf "%.2f" "$min_dist_min"`
	min_dist_min=`echo "$min_dist_min" | tr ",", "."`
	if (( $(echo "$min_dist_min < 0.20" |bc -l) ))
	then
		min_dist_min="\textcolor{red}{$min_dist_min}";
	else
		min_dist_min="\textcolor{ForestGreen}{$min_dist_min}";
	fi;
	
#	max_radius=`echo "$max_radius" | tr ".", ","`
	max_radius=`printf "%.2f" "$max_radius"`
	max_radius=`echo "$max_radius" | tr ",", "."`
	if (( $(echo "$max_radius > 4.0" |bc -l) ))
	then
		max_radius="\textcolor{red}{$max_radius}";
	else
		max_radius="\textcolor{ForestGreen}{$max_radius}";
	fi;

#	min_obstacle=`echo "$min_obstacle" | tr ".", ","`
	min_obstacle=`printf "%.2f" "$min_obstacle"`
	min_obstacle=`echo "$min_obstacle" | tr ",", "."`
	if (( $(echo "$min_obstacle < 0.30" |bc -l) ))
	then
		min_obstacle="\textcolor{red}{$min_obstacle}";
	else
		min_obstacle="\textcolor{ForestGreen}{$min_obstacle}";
	fi;
	if [ "$obstacles" = "0" ]
	then
		min_obstacle="-"
	fi;
	
	
	
	if [ "$controller" = "A" ]
	then
		echo "$ndrones & \$$neighbourhood_distance\$ & $obstacles & $min_dist_min & $max_radius & $min_obstacle & % mode:$mode controller:$controller "
	fi;
	if [ "$controller" = "B" ]
	then
	echo " $min_dist_min & $max_radius & $min_obstacle \\\\ % mode:$mode controller:$controller "
	fi;


done < "data_sorted.csv"

echo " "
echo " "
echo " "
echo " "
echo "=========================="

echo " "
echo " "
echo " "
echo " "
echo "calling Make:"
cd docker
./metrics.py $hash obstacle2 gradenum _A_
./metrics.py $hash obstacle2 gradenum _B_
cd ..

exit 0

echo "% %%%% simulation hash: $hash %%%%"
while read -r line
do
	obstacles=`echo "$line" | cut -d "," -f 1 | bc`
	neighbourhood_distance=`echo "$line" | cut -d "," -f 2`
	ndrones2=`echo "$line" | cut -d "," -f 3`
	ndrones=`echo "$line" | cut -d "," -f 4`
	mode=`echo "$line" | cut -d "," -f 5`
	controller=`echo "$line" | cut -d "," -f 6`
	min_dist_min=`echo "$line" | cut -d "," -f 7`
	max_radius=`echo "$line" | cut -d "," -f 8`
	min_obstacle=`echo "$line" | cut -d "," -f 9`

	if [ "$neighbourhood_distance" = "999.0" ]
	then
		continue; ## do not include global neighborhood
		neighbourhood_distance="\infty"
	fi;
	
	if [ "$controller" = "1" ]
	then
		controller="A"
	fi;
	if [ "$controller" = "2" ]
	then
		controller="B"
	fi;

	min_dist_min=`printf "%.2f" "$min_dist_min"`
	min_dist_min=`echo "$min_dist_min" | tr ",", "."`
	if (( $(echo "$min_dist_min < 0.20" |bc -l) ))
	then
		min_dist_min="\textcolor{red}{$min_dist_min}";
	else
		min_dist_min="\textcolor{ForestGreen}{$min_dist_min}";
	fi;
	
	max_radius=`printf "%.2f" "$max_radius"`
	max_radius=`echo "$max_radius" | tr ",", "."`
	if (( $(echo "$max_radius > 10.0" |bc -l) ))
	then
		max_radius="\textcolor{red}{$max_radius}";
	else
		max_radius="\textcolor{ForestGreen}{$max_radius}";
	fi;

	min_obstacle=`printf "%.2f" "$min_obstacle"`
	min_obstacle=`echo "$min_obstacle" | tr ",", "."`
	if (( $(echo "$min_obstacle < 0.28" |bc -l) ))
	then
		min_obstacle="\textcolor{red}{$min_obstacle}";
	else
		min_obstacle="\textcolor{ForestGreen}{$min_obstacle}";
	fi;
	if [ "$obstacles" = "0" ]
	then
		min_obstacle="-"
		# continue;
	fi;
	
	
	
	if [ "$controller" = "A" -a "$mode" = "gradenum" ]
	then
		echo "$ndrones & ~$obstacles~~ & $min_dist_min & $max_radius & $min_obstacle & % mode:$mode controller:$controller "
	elif [ "$controller" = "B" -a "$mode" = "gradient" ]
	then
		echo " $min_dist_min & $max_radius & $min_obstacle \\\\ % mode:$mode controller:$controller "
	else
		echo " $min_dist_min & $max_radius & $min_obstacle & % mode:$mode controller:$controller "
	fi;


done < "data_sorted2.csv"

echo " "
echo " "
echo " "
echo " "
echo "=========================="

exit 0;

for fn in `ls | grep ${hash}_`
do
	ndrones=`echo $fn | cut -d "_" -f 5 | cut -d "m" -f 2`;
	pushd $fn
	
	cp ../Makefile .
	# rm -f Cleaned*
	# make CleanedAllMetrics$ndrones.png # > /dev/null 2> /dev/null
	#make CleanedAllMetricsAgr$ndrones.csv # > /dev/null 2> /dev/null
	
	popd
done;

echo "=========================="

rm combined_$hash-*.pdf

for fn in `ls | grep ${hash}_ | grep swarm15`
do
	ttype=`echo $fn | grep -o -e "free" -e "obstacle" | cut -b 1-3`
	#fc=`echo $fn | cut -b 41- | cut -b -32`
	fc1=`echo $fn | cut -d "_" -f 6`
	fc2=`echo $fn | cut -d "_" -f 7`
	fc3=`echo $fn | cut -d "_" -f 8`
	fc4=`echo $fn | cut -d "_" -f 9`
	fc5=`echo $fn | cut -d "_" -f 10`
	fc6=`echo $fn | cut -d "_" -f 11`

	fc=${fc1}_${fc2}_${fc3}_
	fb=${fc2}_${fc3}_${fc4}_
	ff=`[ $ttype = "obs" ] && echo $fb || echo  $fc`
	echo $fn" --> "$ff" ("$ttype")"
	#fc=`echo $fn | cut -b 41- | cut -b -32`
	#ttype=""
	
	f15=`ls *$hash*swarm15*$ff*$ttype*/CleanedAllMetrics15.csv`
	f9=`ls *$hash*swarm9*$ff*$ttype*/CleanedAllMetrics9.csv`
	f4=`ls *$hash*swarm4*$ff*$ttype*/CleanedAllMetrics4.csv`
	f2=`ls *$hash*swarm2*$ff*$ttype*/CleanedAllMetrics2.csv`
	echo "f15 = $f15"
	echo "f9 = $f9"
	echo "f4 = $f4"
	echo "f2 = $f2"
	agr15=`ls *$hash*swarm15*$ff*$ttype*/CleanedAllMetricsAgr15.csv`
	agr9=`ls *$hash*swarm9*$ff*$ttype*/CleanedAllMetricsAgr9.csv`
	agr4=`ls *$hash*swarm4*$ff*$ttype*/CleanedAllMetricsAgr4.csv`
	agr2=`ls *$hash*swarm2*$ff*$ttype*/CleanedAllMetricsAgr2.csv`
	echo "agr15 = $agr15"
	echo "agr9 = $agr9"
	echo "agr4 = $agr4"
	echo "agr2 = $agr2"

	./Agregate.py $agr2 $agr4 $agr9 $agr15 > CleanedAgrTmp.csv

	echo " \
		set datafile separator ',' 
		set key out vert
		set key right top
		set key off
		min(a,b) = (a < b) ? a : b 
		max(a,b) = (a < b) ? b : a 
		set term pdf font \"Times,15\"
		set grid 
		set xlabel \"Time [s]\" 
		set ylabel \"Distance [m]\" 
		set xrange [20:75] 
		set yrange [0:3.0] 
		set output \"combined_$hash-$ff${ttype}_Agr.pdf\" 
		plot 	 \"CleanedAgrTmp.csv\" using (column(1)):(column(2)) with lines linewidth 2 linecolor 1 dt 1 title 'dist_{min}', \
			 \"CleanedAgrTmp.csv\" using (column(1)):(column(3)) with lines linewidth 2 linecolor 2 dt 1 title 'comp_{max}', \
			 \"CleanedAgrTmp.csv\" using (column(1)):(column(4)) with lines linewidth 2 linecolor 3 dt 1 title 'clear_{obj}', \
		\
			0.2 with lines linewidth 2 lt rgb 'light-red' dashtype 2 title 'dist_{thr} (20cm)', \
			0.3 with lines linewidth 2 lt rgb 'coral' dashtype 2 title 'clear_{thr} (30cm)'" \
	 | gnuplot

	echo " \
		set datafile separator ',' 
		set key out vert
		set key right top
		set key off
		min(a,b) = (a < b) ? a : b 
		max(a,b) = (a < b) ? b : a 
		set term pdf font \"Times,15\"
		set grid 
		set xlabel \"Time [s]\" 
		set ylabel \"Distance [m]\" 
		set xrange [20:75] 
		set yrange [0:3.0] 
		set output \"combined_$hash-$ff${ttype}_Test.pdf\" 
		plot     \"$agr15\" using (column(1)+column(2)/1000000000):(column(4)) with lines linewidth 2 linecolor 1 dt 1 title '15 drones, dist_{min}', \
			 \"$agr15\" using (column(1)+column(2)/1000000000):(column(5)) with lines linewidth 2 linecolor 2 dt 1 title '15 drones, comp_{max}', \
			 \"$agr15\" using (column(1)+column(2)/1000000000):(column(6)) with lines linewidth 2 linecolor 3 dt 1 title '15 drones, clear_{obj}', \
		\
			 \"$agr9\" using (column(1)+column(2)/1000000000):(column(4)) with lines linewidth 2 linecolor 'purple' dt 1 title '9 drones, dist_{min}', \
			 \"$agr9\" using (column(1)+column(2)/1000000000):(column(5)) with lines linewidth 2 linecolor 'web-green' dt 1 title '9 drones, comp_{max}', \
			 \"$agr9\" using (column(1)+column(2)/1000000000):(column(6)) with lines linewidth 2 linecolor 'blue' dt 1 title '9 drones, clear_{obj}', \
		\
			 \"$agr4\" using (column(1)+column(2)/1000000000):(column(4)) with lines linewidth 2 linecolor 'orchid' dt 1 title '4 drones, dist_{min}', \
			 \"$agr4\" using (column(1)+column(2)/1000000000):(column(5)) with lines linewidth 2 linecolor 'light-green' dt 1 title '4 drones, comp_{max}', \
			 \"$agr4\" using (column(1)+column(2)/1000000000):(column(6)) with lines linewidth 2 linecolor 'skyblue' dt 1 title '4 drones, clear_{obj}', \
		\
			\"$agr2\" using (column(1)+column(2)/1000000000):(column(4)) with lines linewidth 2 linecolor 'magenta' dt 1 title '2 drones, dist_{min}', \
		   	\"$agr2\" using (column(1)+column(2)/1000000000):(column(5)) with lines linewidth 2 linecolor 'sea-green' dt 1 title '2 drones, comp_{max}', \
			\"$agr2\" using (column(1)+column(2)/1000000000):(column(6)) with lines linewidth 2 linecolor 'dark-blue' dt 1 title '2 drones, clear_{obj}', \
		\
			0.2 with lines linewidth 2 lt rgb 'light-red' dashtype 2 title 'dist_{thr} (20cm)', \
			0.3 with lines linewidth 2 lt rgb 'coral' dashtype 2 title 'clear_{thr} (30cm)'" \
	 | gnuplot

	echo " \
		set datafile separator ',' 
		set key out vert
		set key right top
		set key off
		min(a,b) = (a < b) ? a : b 
		max(a,b) = (a < b) ? b : a 
		set term pdf font \"Times,15\"
		set grid 
		set xlabel \"Time [s]\" 
		set ylabel \"Distance [m]\" 
		set xrange [20:75] 
		set yrange [0:3.0] 
		set output \"combined_$hash-$ff$ttype.pdf\" 
		plot     \"$f15\" using (column(1)+column(2)/1000000000):(min(column(4),min(column(7),min(column(10),min(column(13),min(column(16),min(column(19),min(column(22),min(column(25),min(column(28),min(column(31),min(column(34),min(column(37),min(column(40),min(column(43),column(46)))))))))))))))) with lines linewidth 2 linecolor 1 dt 1 title '15 drones, dist_{min}', \
			 \"$f15\" using (column(1)+column(2)/1000000000):(max(column(5),max(column(8),max(column(11),max(column(14),max(column(17),max(column(20),max(column(23),max(column(26),max(column(29),max(column(32),max(column(35),max(column(38),max(column(41),max(column(44),column(47)))))))))))))))) with lines linewidth 2 linecolor 2 dt 1 title '15 drones, comp_{max}', \
			 \"$f15\" using (column(1)+column(2)/1000000000):(min(column(6),min(column(9),min(column(12),min(column(15),min(column(18),min(column(21),min(column(24),min(column(27),min(column(30),min(column(33),min(column(36),min(column(39),min(column(42),min(column(45),column(48)))))))))))))))) with lines linewidth 2 linecolor 3 dt 1 title '15 drones, clear_{obj}', \
		\
			 \"$f9\" using (column(1)+column(2)/1000000000):(min(column(4),min(column(7),min(column(10),min(column(13),min(column(16),min(column(19),min(column(22),min(column(25),column(28)))))))))) with lines linewidth 2 linecolor 'purple' dt 1 title '9 drones, dist_{min}', \
			 \"$f9\" using (column(1)+column(2)/1000000000):(max(column(5),max(column(8),max(column(11),max(column(14),max(column(17),max(column(20),max(column(23),max(column(26),column(29)))))))))) with lines linewidth 2 linecolor 'web-green' dt 1 title '9 drones, comp_{max}', \
			 \"$f9\" using (column(1)+column(2)/1000000000):(min(column(6),min(column(9),min(column(12),min(column(15),min(column(18),min(column(21),min(column(24),min(column(27),column(30)))))))))) with lines linewidth 2 linecolor 'blue' dt 1 title '9 drones, clear_{obj}', \
		\
			 \"$f4\" using (column(1)+column(2)/1000000000):(min(column(4),min(column(7),min(column(10),column(13))))) with lines linewidth 2 linecolor 'orchid' dt 1 title '4 drones, dist_{min}', \
			 \"$f4\" using (column(1)+column(2)/1000000000):(max(column(5),max(column(8),max(column(11),column(14))))) with lines linewidth 2 linecolor 'light-green' dt 1 title '4 drones, comp_{max}', \
			 \"$f4\" using (column(1)+column(2)/1000000000):(min(column(6),min(column(9),min(column(12),column(15))))) with lines linewidth 2 linecolor 'skyblue' dt 1 title '4 drones, clear_{obj}', \
		\
			\"$f2\" using (column(1)+column(2)/1000000000):(min(column(4),column(7))) with lines linewidth 2 linecolor 'magenta' dt 1 title '2 drones, dist_{min}', \
		   	\"$f2\" using (column(1)+column(2)/1000000000):(max(column(5),column(8))) with lines linewidth 2 linecolor 'sea-green' dt 1 title '2 drones, comp_{max}', \
			\"$f2\" using (column(1)+column(2)/1000000000):(min(column(6),column(9))) with lines linewidth 2 linecolor 'dark-blue' dt 1 title '2 drones, clear_{obj}', \
		\
			0.2 with lines linewidth 2 lt rgb 'light-red' dashtype 2 title 'dist_{thr} (20cm)', \
			0.3 with lines linewidth 2 lt rgb 'coral' dashtype 2 title 'clear_{thr} (30cm)'" \
	 | gnuplot


	echo $fn
done;


