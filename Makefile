# definitions

# rules
berry01: berry01.o
	echo "\nLinking...\n"
	g++ -lm -li2c berry01.o 

berry01.o: berry01.cpp berryimu.o
	echo "\nCompiling berry01\n"
	g++ -c  -o berry01.o berry01.cpp

berryimu.h.gch: berryimu.h berryimuDefinitions.h
	echo "\nCompiling berryimu\n"
	g++ -c -o berryimu.h.gch berryimu.h


#berryimu.cpp: berryimu.h
#	echo "berryimu.h was changed"
