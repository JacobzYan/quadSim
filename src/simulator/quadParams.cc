
// Class to store/get params
// Add function to pull params from some external file, define interfaces
#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <fstream>

#include "quadParams.h"
#include "poseModules.h"
#include "utils.h"



// ===== quadParams Implementations =====
void quadParams::setParams(const std::vector<std::string> configPaths)
{
    // Read in each file
    int nFiles = configPaths.size();
    for(int i=0;i<nFiles;i++){readFile(configPaths[i]);}
}

bool quadParams::readFile(std::string path)
    {
        // Read in file, check that file exists
        std::ifstream file(path);
        if (!file.is_open())
        {
            std::cout << "FAILED TO OPEN FILE: " << std::endl << path << std::endl;
            return false;
        }
        
        // Iterate through lines
        while(std::getline(file, line))
        {
            // Skip Comments, empty lines
            if(line.size() == 0 || line[0] == '#'){continue;}
            
            // Split line by equals
            delimiterLocation = line.find("=");
            varName = line.substr(0,delimiterLocation);
            varValue = line.substr(delimiterLocation+1, line.size()-delimiterLocation-1);

            // Trim name whitespace
            cutWhitespace(varName);
            
            // Ensure there is an equals sign
            if(delimiterLocation==std::string::npos)
            {
                std::cout << "THIS LINE CONTAINS NO EQUALS SIGN DELIMITER:" << std::endl << line << std::endl;
                return false;
            }

            // Check if line start matches any variable names
            varIndex = -1;
            for(int i=0;i<nVars;i++)
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
                case 0: g(std::stod(varValue)); break;
                case 1: pAir(std::stod(varValue)); break;
                case 2: m(std::stod(varValue)); break;
                case 3: Cd(std::stod(varValue)); break;
                case 4: Ad(std::stod(varValue)); break;
                case 5: J(splitMatrix3d(varValue, ',')); break;
                case 6: xCOM(splitVector3d(varValue, ',')); break;
                case 7: addProp(new propParams(varValue)); break;
                case 8: name(varValue); break;
                default: std::cout << "Could not read line - variable name not recognized: " << line << std::endl;
            }
        }
        
        return true;
    }

// Print Functions
void quadParams::printValues() const {printValues(0);} // Default prints with no tabs
void quadParams::printValues(int nTabs) const
{
    using std::endl;
    const std::string startingTabs = repeatStr("\t", nTabs);
    const std::string matLineStart = "\n\t\t"+startingTabs;
    Eigen::IOFormat matrixFormat{Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", matLineStart};

    std::cout
    << startingTabs << name_ << " Parameters:" << endl
    << startingTabs << "\tg: " << g() << endl
    << startingTabs << "\tpAir: " << pAir() << endl
    << startingTabs << "\tm: " << m() << endl
    << startingTabs << "\tCd: " << Cd() << endl
    << startingTabs << "\tAd: " << Ad() << endl
    << startingTabs << "\tJ: " << matLineStart << J().format(matrixFormat) << endl
    << startingTabs << "\txCOM: " << matLineStart << xCOM().format(matrixFormat) << endl
    ;

    for(const propParams * prop : props_)
    {
        prop->printValues(nTabs+1);
    }
}


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
        // Split line by semicolon
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
        for(int i=0;i<nVars;i++)
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
            case 2: // kMotor
                kMotor(std::stod(varValue));
                break;
            case 3: // kF
                kF(std::stod(varValue));
                break;
            case 4: // kN
                kN(std::stod(varValue));
                break;
            case 5: // omegaRDir
                omegaRDir(std::stod(varValue));
                break;
            case 6: // cm
                cm(std::stod(varValue));
                break;
            case 7: // tauM
                tauM(std::stod(varValue));
                break;
            case 8:
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
    << startingTabs << "\tkMotor: " << kMotor() << endl
    << startingTabs << "\tkF: " << kF() << endl
    << startingTabs << "\tkN: "  << kN() << endl
    << startingTabs << "\tdirection: "  << omegaRDir() << endl
    << startingTabs << "\tcm: "  << cm() << endl
    << startingTabs << "\ttauM: "  << tauM() << endl
    ;
}