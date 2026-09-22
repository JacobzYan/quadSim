// #pragma once
#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <fstream>

#include "poseModules.h"
#include "utils.h"

// ===== poseTemplate Implementations =====

// ===== propParams Implementations =====
propParams::propParams(const std::string & line)
{
    // Temp helper variables
    std::string varName;
    std::string varValue; 
    int delimiterLocation;
    int varIndex;

    // Process line
    std::string processingLine = line;
    cutWhitespace(processingLine);

    // Seperate out prop parameters with semicolon delimeters
    replaceDelimiters(processingLine,';');
    std::istringstream iss(processingLine);

    std::string packet;
    while(iss >> packet)
    {
        // Seperate out prop parameters
        delimiterLocation = packet.find("=");
        varName = packet.substr(0,delimiterLocation);
        varValue = packet.substr(delimiterLocation+1, packet.size()-delimiterLocation-1);

        // Trim all whitespace
        cutWhitespace(varName);

        // Ensure there is an equals sign
        if(delimiterLocation==std::string::npos)
        {
            std::cout << "THIS PACKET CONTAINS NO EQUALS SIGN DELIMITER:" << std::endl << packet << std::endl;
            continue;
        }

        // Check if line start matches any variable names
        varIndex = -1;
        for(int i=0;i<varNames.size();i++)
        {
            if(varNames[i] == varName)
            {
                varIndex = i;
                break;
            }
        }

        // Trim Whitespace for name, cut all whitespace for others
        if(varIndex==7){trim(varValue);} // Name - preserve internal spaces
        else{cutWhitespace(varValue);}

        // Assign the appropriate value
        switch(varIndex)
        {
            case 0: // location
                location(splitVector3d(varValue, ','));
                break;
            case 1: // R
                R(splitMatrix3d(varValue, ','));
                break;
            case 2: // kF
                kF(std::stod(varValue));
                break;
            case 3: // kN
                kN(std::stod(varValue));
                break;
            case 4: // omegaRDir
                omegaRDir(std::stod(varValue));
                break;
            case 5: // cm
                cm(std::stod(varValue));
                break;
            case 6: // tauM
                tauM(std::stod(varValue));
                break;
            case 7: // name
                name(varValue);
                break;
            default:
                break;
        }
    }
}

// Print Functions
void propParams::printValues() const {printValues(0);}
void propParams::printValues(int nTabs) const
{
    using std::endl;
    std::string startingTabs = repeatStr("\t", nTabs);
    const std::string matLineStart = "\n\t\t"+startingTabs;
    Eigen::IOFormat matrixFormat{Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n\t\t"+startingTabs};

    std::cout
    << startingTabs << name_ << " Parameters:" << endl
    << startingTabs << "\tlocation: " << matLineStart << location().format(matrixFormat) << endl
    << startingTabs << "\tR: " << matLineStart << R().format(matrixFormat) << endl
    << startingTabs << "\tkF: " << kF() << endl
    << startingTabs << "\tkN: "  << kN() << endl
    << startingTabs << "\tdirection: "  << omegaRDir() << endl
    << startingTabs << "\tcm: "  << cm() << endl
    << startingTabs << "\ttauM: "  << tauM() << endl
    ;
}
