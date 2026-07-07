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
echo "Got here"
cd ../bin
echo "Got here"
cd testers/testSim/ 
./simTest
echo "Got here"
# Copy data over
cp outputData.csv ../../../automations/outputs/testSimOutputData.csv
cp defaultQuad.param ../../../automations/outputs/testSimDefaultQuad.param
cd ../../..


# Visualize in python
echo visualizing...
source quadSimPythonVenv/QSVenv/bin/activate

cd automations
python3 helpers/visualizeQuad.py outputs/testSimDefaultQuad.param outputs/testSimOutputData.csv outputs/testSimVisual.mp4

# Play video
vlc outputs/testSimVisual.mp4  2>/dev/null # Play the video on vlc without including error messages about not being able to hw accelerate
