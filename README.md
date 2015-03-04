# BA_with_semantic_constraints

The code for Bundle adjustment with constraints

Installation:

Ceres 

git clone https://ceres-solver.googlesource.com/ceres-solver
cd ceres-solver
mkdir ceres-bin
cd ceres-bin
cmake ../ceres-solver-1.10.0
make -j3
make test
make install

Wrapper for ceres

git clone https://github.com/dineshreddy91/constrained_BA
cd constrained_BA
cmake .
make

To run the algorithm we use matlab

The main file is box_synthetic.m



