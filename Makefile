COMPILER = g++
INCLUDE = /opt/homebrew/Cellar/eigen/3.4.0_1/include/eigen3

gradient-descent: fk.cpp ik-gradient.cpp
	${COMPILER} -o ik-gradient ik-gradient.cpp -I${INCLUDE}
	${COMPILER} -o ik-newton-raphson ik-newton-raphson.cpp -I${INCLUDE}
	${COMPILER} -o fk fk.cpp -I${INCLUDE}
