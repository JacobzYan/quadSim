Eigen Datatypes

# Vector
- Syntax: ```Eigen::Vector<XX,YY>```
	- XX -> length
	- YY -> Datatype
- Notes:
	- Vectors are col vectors by default
~~~ Example_uses
# Datatype Examples
Eigen::Vector<3, double> # Length 3 vector of doubles
Eigen::Vector4f # Length 4 vector of floats

# Use Examples
Eigen::Vector3d Vec = Eigen::Vector::Zeros(); // Initializes the vector with all values as zeros
Vec = Eigen::Vector::Ones(); # Reassigns all the values to ones
Vec(0) = 3.0; # Reassign the first value to 3
Vec << 5,6,7; # Re-assign all values to the vector - must pipe in all values
Eigen::Vector2d Vec2 = Vec.segment(1,2); # Init Vec2 and assign it the values of Vec starting at index 1
~~~

# Matrix
Syntax: ```Eigen::Matrix<XX, YY, ZZ>```
- XX -> num Rows
- YY -> num Cols
- ZZ -> datatype
- Notes:
	- can use .array() to do elementwise operations
~~~ Example_uses
# Datatype Examples
Eigen::Matrix<5,2, double> # 5x2 matrix of doubles
Eigev::Matrix3d # 3x3 matrix of doubles

# Use Examples
TBD


~~~
