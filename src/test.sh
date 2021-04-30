#!/bin/sh
SAMPLE="../sample_circuits"
PATTERN="../tdf_patterns"
MY="./atpg -tdfsim"
TA="../bin/golden_tdfsim -tdfsim"
MYOUT="../myOut"
TAOUT="../taOut"

for FILE in "$SAMPLE"/*.ckt
do
	NAME=$(echo $FILE | awk -F '[/]' '{print $3}' | awk -F '[.]' '{print $1}')
	$MY $PATTERN/$NAME.pat $SAMPLE/$NAME.ckt > $MYOUT/$NAME.dat
	$TA $PATTERN/$NAME.pat $SAMPLE/$NAME.ckt > $TAOUT/$NAME.dat
	diff $MYOUT/$NAME.dat $TAOUT/$NAME.dat > ../diff/${NAME}_diff.dat
done
