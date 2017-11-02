#!/bin/sh
echo "start"
b=1
DATEIENDUNG=txt

until [ $b -gt 4 ]
do 
	a=70
	eu=`echo "scale=5; $b * 0.001" | bc`

	until [ $a -gt 160 ]
	do
	NEWNAME="euler"$a"_0"
	./Assignment1 -method euler -stiff $a -step $eu -damp 0.1 -mass 0.15  & epid=$!
	(sleep 20 && kill -9 $epid) 
	cp lastrun_0.log  $(printf $NEWNAME$eu).$DATEIENDUNG
	a=`expr $a + 10`
	
	done
	
	b=`expr $b + 1`
	
done

