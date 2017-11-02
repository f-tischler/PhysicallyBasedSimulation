#!/bin/sh
echo "start"
b=7
DATEIENDUNG=txt

until [ $b -gt 10 ]
do 
	a=70
	eu=`echo "scale=5; $b * 0.0075" | bc`

	until [ $a -gt 160 ]
	do
	NEWNAME="symplectic"$a"_0"
	./Assignment1 -method symplectic -stiff $a -step $eu -damp 0.1 -mass 0.15  & epid=$!
	(sleep 20 && kill -9 $epid) 
	cp lastrun_1.log  $(printf $NEWNAME$eu).$DATEIENDUNG
	a=`expr $a + 30`
	
	done
	
	b=`expr $b + 1`
	
done

