#/bin/bash

function Robot_Sturn {

	echo ENGS $1
	rslt=""
	while [[ "$rslt" != COLOR[TF]Sturn ]]
	do
		read rslt
	done
	if [[ "$rslt" == COLORFSturn ]]
	then
		#probleme de positionnement
		return 0
	fi
	return 1
}

