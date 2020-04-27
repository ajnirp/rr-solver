all: rr-solver

rr-solver: rr-solver.cpp
	g++ rr-solver.cpp -o rr-solver

clean:
	rm -f rr-solver