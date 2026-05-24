# Save current dir
CURRENT_DIR=${pwd}

# Check if want to reset make files, recompile
if [ "$1" = "-c" ]; then
    echo "Remaking make files and recompiling project"
    cd ../build
    cmake ..
    make
fi

# Check if want to recompile
if [ "$1" = "-m" ]; then
    # Code to run if -m flag is passed
    echo "Recompiling project"
    cd ../build
    make
fi



# Run the ODE tester
cd ../bin
./odeTest
cd ..


# Visualize in python
echo visualizing...
source quadSimVenv/bin/activate


python3 automations/helpers/visualizeOdeTester.py bin/output.csv