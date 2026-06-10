#pragma once
#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <fstream>


// Strips out all whitespace from a variable inplace
void cutWhitespace(std::string& str) {
    std::string result;
    result.reserve(str.size());  // Avoid reallocations
    
    for (char ch : str) {
        if (!std::isspace(static_cast<unsigned char>(ch))) {
            result += ch;
        }
    }
    
    str = std::move(result);
}


// Cuts whitespace from the beginning and end of a string inplace
void trim(std::string &str)
{
    const char* whitespace = " \t\n\r\f\v";

    str.erase(0, str.find_first_not_of(whitespace));
    str.erase(str.find_last_not_of(whitespace) + 1);
}


// Replaces all instances of a delimiter in a string with a space, Returns the number of delimiters that were replaced
int replaceDelimiters(std::string & str, const char & delimiter)
{
    int nReplaced = 0;
    cutWhitespace(str);
    int delimiterIndex;
    while(delimiterIndex = str.find(delimiter) != std::string::npos)
    {
        delimiterIndex = str.find(delimiter);
        nReplaced += 1;
        str.replace(delimiterIndex, 1, 1, ' ');
    }
    return nReplaced;
}


// Takes a string with numbers seperated by commas, and  returns an eigen vector3d
const Eigen::Vector3d splitVector3d(const std::string & line, const char delimiter)
{
    double vectorData[3];
    int assignmentIndex = 0;
    // Replace commas with whitespace
    std::string data = line;
    
    const int nCommmas = replaceDelimiters(data, delimiter);

    // Ensure number of commas is correct
    if(nCommmas < 2){std::cout << "INSUFFICIENT VALUES PASSED TO SPLITVECTOR3D: " << line << std::endl; return Eigen::Vector3d::Zero();}
    if(nCommmas > 2){std::cout << "TOO MANY VALUES PASSED TO SPLITVECTOR3D: " << line << std::endl; return Eigen::Vector3d::Zero();}

    // Steam line
    std::istringstream iss(data);
    double value;
    // populate vectorData
    while(iss >> value){vectorData[assignmentIndex] = value; assignmentIndex ++;}
    Eigen::Map<Eigen::Vector3d> vec(&vectorData[0]);
    return vec;
}


// Takes a string with numbers seperated by commas, and  returns an eigen matrix3d
const Eigen::Matrix3d splitMatrix3d(const std::string & line, const char delimiter)
{
    double vectorData[9];
    int assignmentIndex = 0;
    // Replace commas with whitespace
    std::string data = line;
    
    int nCommmas = replaceDelimiters(data, delimiter);

    // Ensure number of commas is correct
    if(nCommmas < 8){std::cout << "INSUFFICIENT VALUES PASSED TO SPLITMATRIX3D: " << line << std::endl; return Eigen::Matrix3d::Zero();}
    if(nCommmas > 8){std::cout << "TOO MANY VALUES PASSED TO SPLITMATRIX3D: " << line << std::endl; return Eigen::Matrix3d::Zero();}

    // Steam line
    std::istringstream iss(data);
    double value;
    // populate vectorData
    while(iss >> value){vectorData[assignmentIndex] = value; assignmentIndex ++;}
    Eigen::Map<Eigen::Matrix3d> vec(&vectorData[0]);
    return vec;

}


// Concatenates a string to itself n times
std::string repeatStr(const std::string & str, const int n)
{
    std::string output = "";
    for(int i=0;i<n;i++){output += str;}
    return output;
}


// IMPLEMENT LATER
Eigen::Vector4d AA2Quat(const Eigen::Vector3d & axis, const double angle) // Converts axis angle representation to a quaternion
{
    return Eigen::Vector4d::Zero();
}

