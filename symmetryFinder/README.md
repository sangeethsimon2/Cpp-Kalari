This piece of code takes as input a set of points in the form of 
a std::vector<> and checks for possible symmetry in it and if 
there exists one, returns the line containing this symmetry.

This version implements a fast search for a symmetry among the point cloud that:
-> Processes the midpoints and perpendicular bisector lines of every pair of points and stores them in unordered_sets using an integer hash map
-> Employs an algorithm to identify only the most promising candidate lines of symmetry 
-> Checks the whole point cloud's symmetry about these potential lines of symmetries only.

This version is O(N^3) complexity in the worst case but could be much faster on practical cases thanks to its avoidance 
of nested for loops over all point pairs to check if their perpendicular bisectors are potential lines of symmetries, 
and its technique to filter the whole map into a smaller hash map containing only the  most likelihood lines of symmetries.

To compile and run this code, you need:
-> A resonably new GCC compiler (>=11.2.0)
-> CMake 3.23
and optionally
-> Gtest suite 1.14.0 (if you want to run tests) 
 


