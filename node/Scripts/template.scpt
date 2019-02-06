#/bin/bash

outfile=Scripts/TMPLT.OUT


while true
do
	echo "SR1 90"
	sleep 2
	echo "CMP"
	angle=""

	while [[ "$angle" != AXY* ]]
	do
		read angle
	done
	echo ${angle#*"AXY "} >> $outfile 

	echo "SR1 0"
	sleep 2


done
echo "END"
