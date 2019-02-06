#/bin/bash

outfile=Scripts/TMPLT.OUT

source Scripts/RobotFunctions.scpt

######

function mesure {
	#orienter dans la direction et attendre bon positionnement
	
	Robot_Sturn $1
	if [ $? -eq 0 ]; then #probleme de positionnement		
		echo "ENDF"
		exit
	fi

	echo "TOPW" # fonction pas encore normalisée danc pas dans RobotFunctions
	while [[ "$rslt" != TOPW* ]]
	do
		read rslt
	done
	if [[ "$rslt" == TOPWF ]]
	then
		#probleme de mesure
		echo "ENDF"
		exit
	fi
}

mesure 0
sleep 3
mesure 90
sleep 3
mesure 180
sleep 3
mesure 270


echo "ENDT"
