# definitions

# rules
berry01: berry01.o
	echo "\nLinking...\n\n\n"
	g++ -lm -li2c -lrt berry01.o 

berry01.o: berry01.cpp berryimu.h.gch
	echo "\nCompiling berry01:\n\n\n"
	g++ -c  -o berry01.o berry01.cpp

berryimu.h.gch: berryimu.h berryimuDefinitions.h
	echo "\nPrecompiling berryimu header file:\n\n\n"
	g++ -c -o berryimu.h.gch berryimu.h


#berryimu.cpp: berryimu.h
#	echo "berryimu.h was changed"
