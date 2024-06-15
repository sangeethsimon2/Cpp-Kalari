This piece of code takes as input a set of points in the form of
a std::vector<> and checks for possible symmetry in it and if
there exists one, returns the line containing this symmetry.

This version implements a search method for finding a symmetry among the point cloud that:
- Processes the midpoints and perpendicular bisector lines of every pair of points
- Checks the whole point cloud's symmetry about these potential lines of symmetries only.
- The symmetry check is done by storing the points in an unordered_map using integer hashing, 
finding symmetry points and then searching for these points in the map to verify symmetry. 

This version is O(N^3) complexity in the best case.

To compile and run this code, you need:
- A resonably new GCC compiler (>=11.2.0)
- CMake 3.23
and optionally
- Gtest suite 1.14.0 (if you want to run tests)


