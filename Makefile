COMPILER = g++
INCLUDE = /opt/homebrew/Cellar/eigen/3.4.0_1/include/eigen3

gradient-descent:
	${COMPILER} -o ik-geometric src/ik-geometric.cpp -I${INCLUDE}
	${COMPILER} -o ik-gradient src/ik-gradient.cpp -I${INCLUDE}
	${COMPILER} -o ik-newton-raphson src/ik-newton-raphson.cpp -I${INCLUDE}
	${COMPILER} -o fk src/fk.cpp -I${INCLUDE}

clean:
	rm ik-geometric
	rm ik-gradient
	rm ik-newton-raphson
	rm fk
