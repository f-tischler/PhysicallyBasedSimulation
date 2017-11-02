#!/bin/sh
echo "start"
b=5
DATEIENDUNG=txt

until [ $b -gt 8 ]
do 
	a=70
	eu=`echo "scale=5; $b * 0.01" | bc`

	until [ $a -gt 160 ]
	do
	NEWNAME="midpoint"$a"_0"
	./Assignment1 -method midpoint -stiff $a -step $eu -damp 0.1 -mass 0.15  & epid=$!
	(sleep 20 && kill -9 $epid) 
	cp lastrun_3.log  $(printf $NEWNAME$eu).$DATEIENDUNG
	a=`expr $a + 10`
	
	done
	
	b=`expr $b + 1`
	
done

