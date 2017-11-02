#!/bin/bash
b=5
DATEIENDUNG=txt

until [ $b -gt 8 ]
do 
	a=70
	eu=`echo "scale=5; $b * 0.01" | bc`

	until [ $a -gt 160 ]
	do
	NEWNAME="midpoint"
	
		gnuplot -persist <<-EOFMarker
			set xlabel "t[s]"
			set ylabel "Normalized RMS Error"
			set datafile separator ";"
			set style line 1 lc rgb '#0060ad' lt 1 lw 2 pt 7 ps 1.5
			set decimalsign locale; set decimalsign "."
			set terminal jpeg
			set output "$(printf $NEWNAME$a)_0$eu.jpg"
			plot '$(printf $NEWNAME$a)_0$eu.$DATEIENDUNG' with lines title "method=$NEWNAME | dt=$eu | stiffness=$a "
		EOFMarker
		
		a=`expr $a + 10`
	
	done
	
	b=`expr $b + 1`
	
done

