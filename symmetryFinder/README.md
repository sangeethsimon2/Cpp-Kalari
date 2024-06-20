This piece of code takes as input a set of points in the form of
a std::vector<> and checks for possible symmetry in it and if
there exists one, returns the line containing this symmetry.

This version implements a fast search for a symmetry among the point cloud that:
- Centers the data and computes its centroid
- Constructs an Eigen MatrixXd of size 2XN and performs a SVD using Eigen header-only library
- The column vectors of U comprises the primary directions to search for symmetry, with usually the second column
  being a strong candidate since it describes an eigenvector orthogonal to the eigenvector with max variation in data
- Checks the whole point cloud's symmetry about these potential lines of symmetries only.

This version is O(N) complexity in the worst case thanks to its dependence on the SVD operation which is
in general of order O(min(mn^2,m^2n)) for a matrix of mXn size. Plus, the symmetry checker operation is also O(N)
due to the use of hash maps for easy look ups.

To compile and run this code, you need:
- A resonably new GCC compiler (>=11.2.0)
- CMake 3.23
and optionally
- Gtest suite 1.14.0 (if you want to run tests)


